"""Assemble live production functions into a hardware-free actual-SDK C test."""
from pathlib import Path
import re

MOCK_HEADERS = {
    'main.h': '#pragma once\n#include <stdint.h>\nuint32_t HAL_GetTick(void);\n',
    'persistent_store.h': '''#pragma once
#include <stddef.h>
#include <stdint.h>
typedef int launchcore_persist_status_t;
#define LAUNCHCORE_PERSIST_OK 0
#define LAUNCHCORE_PERSIST_NOT_FOUND 1
#define LAUNCHCORE_PERSIST_STORAGE_ERROR 2
int persistent_store_init(void);
int persistent_store_get(uint32_t,void*,size_t*);
int persistent_store_set(uint32_t,const void*,size_t);
''',
    'tx_api.h': '#pragma once\n/* Host kernel adapter types precede the actual board mutex helper. */\n',
}


def masked(source):
    return re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
                  lambda m: ''.join('\n' if c == '\n' else ' ' for c in m[0]),
                  source, flags=re.S)


def function(source, name):
    text = masked(source)
    match = re.search(r'(?m)^[A-Za-z_][A-Za-z_0-9 \t*]*\b' + re.escape(name) +
                      r'\s*\([^;{}]*\)\s*\{', text)
    if not match:
        raise AssertionError('Production function missing: ' + name)
    start = text.index('{', match.start())
    top_close = re.search(r'(?m)^}', text[start:])
    limit = start + top_close.end() if top_close else len(text)
    end, depth = start + 1, 1
    while depth and end < limit:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    if depth:
        # DAQ's mutually exclusive preprocessor branches open the same if
        # several times in raw source. Its function closes at column zero.
        if not top_close:
            raise AssertionError('Unterminated production function: ' + name)
        end = limit
    return source[match.start():end]


def assemble(root):
    root = Path(root)
    source = (root / 'Core/Src/telemetry.c').read_text()
    hook_name = 'fchooks.c' if (root / 'Core/Src/fchooks.c').is_file() else 'telemetry_hooks.c'
    hooks = (root / 'Core/Src' / hook_name).read_text()
    program = (root / 'tests/native/router_lifecycle.in.c').read_text()
    for kind, name in re.findall(r'@@(HOOK|TELEMETRY):([A-Za-z_][A-Za-z_0-9]*)@@', program):
        program = program.replace('@@' + kind + ':' + name + '@@',
                                  function(hooks if kind == 'HOOK' else source, name))
    local = re.search(r'(?:static\s+)?const\s+SedsLocalEndpointDesc locals\[\]', source)
    if not local:
        raise AssertionError('Actual endpoint descriptor list missing')
    local_text = source[local.start():source.index('};', local.start()) + 2]
    if '@@GLOBAL_ENDPOINTS@@' in program:
        program = program.replace('@@GLOBAL_ENDPOINTS@@', local_text)
    declarations = ['#define ' + name + ' inert' for name in
                    re.findall(r'\.packet_handler\s*=\s*(\w+)', local_text) if name != 'NULL']
    constants = ('BOARD_CAN_MAX_FRAME_BYTES', 'BOARD_SIDE_TRANSPORT_TEMPLATES',
                 'FC_CAN_MAX_FRAME_BYTES', 'FC_SIDE_TRANSPORT_TEMPLATES',
                 'RF_SEDSNET_QUEUE_BUDGET', 'RF_SEDSNET_MAX_RECENT_RX_IDS',
                 'RF_SEDSNET_STARTING_ALLOCATION', 'RF_CAN_MAX_FRAME_BYTES',
                 'RF_RADIO_MAX_FRAME_BYTES', 'RF_CAN_SIDE_TRANSPORT_TEMPLATES',
                 'RF_RADIO_SIDE_TRANSPORT_TEMPLATES', 'GATEWAY_UART_MAX_FRAME_BYTES',
                 'GATEWAY_SIDE_TRANSPORT_TEMPLATES', 'TELEMETRY_ROUTER_RETRY_MS',
                 'CAN_TX_FRAGMENT_PAYLOAD_BYTES', 'CAN_TX_ALIGNED_FRAME_BYTES',
                 'TELEMETRY_UART_MAX_PAYLOAD')
    inputs = [source]
    for header in ('can_tx_queue.h', 'telemetry_uart.h'):
        path = root / 'Core/Inc' / header
        if path.is_file():
            inputs.append(path.read_text())
    for text in inputs:
        for name in constants:
            match = re.search(r'(?m)^#define ' + name + r'\s+([^\n]+)', text)
            if match:
                declarations += ['#undef ' + name, '#define ' + name + ' ' + match[1]]
    program = program.replace('@@BOARD_DECLARATIONS@@', '\n'.join(declarations))
    if '@@STARTUP_RETRY_LOOP@@' in program:
        text = masked(source)
        match = re.search(r'while \(init_telemetry_router\(\) != SEDS_OK\)\s*\{', text)
        if not match:
            raise AssertionError('Actual FC startup retry loop missing')
        start, end = text.index('{', match.start()), text.index('{', match.start()) + 1
        depth = 1
        while depth:
            depth += (text[end] == '{') - (text[end] == '}')
            end += 1
        program = program.replace('@@STARTUP_RETRY_LOOP@@', source[match.start():end])
    if '@@' in program:
        raise AssertionError('Unresolved production-source fixture token')
    return program
