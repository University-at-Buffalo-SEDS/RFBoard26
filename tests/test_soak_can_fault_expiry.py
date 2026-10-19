"""Connected CAN keeps its zero-expiry gate around explicit ACK-loss faults."""
from pathlib import Path
import importlib.util
import copy
import unittest
spec = importlib.util.spec_from_file_location('run_full_fault_scope', Path(__file__).resolve().parents[1]/'sim/run_full.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class AckFaultExpiryTests(unittest.TestCase):
    def fixture(self):
        topology=dict(sample_count=12, can_ack_events=[
            dict(node='rf',peripheral='fdcan2',after_sample=3,acknowledged=False),
            dict(node='rf',peripheral='fdcan2',after_sample=4,acknowledged=True)],
            reboots=[dict(node='rf',after_sample=8)], assertions=[])
        layout=dict(execution=dict(memory_probes=[dict(name='can_tx_expired',maximum=0),dict(name='can_tx_pending',maximum=48),dict(name='panics',maximum=0)]))
        return topology,dict(rf=copy.deepcopy(layout),valve=copy.deepcopy(layout))
    def passes(self,checks,values):
        return all((values[c['sample']]<=c['maximum']) if 'sample' in c else
                   (values[c['to_sample']]>=values[c['from_sample']] and
                    values[c['to_sample']]-values[c['from_sample']]<=c['maximum_gain']) for c in checks)
    def test_loss_window_is_bounded_and_every_connected_interval_stays_guarded(self):
        topology,layouts=self.fixture();other=copy.deepcopy(layouts['valve'])
        module.scope_can_ack_fault_expiry(topology,layouts)
        checks=topology['assertions'];baseline=[0,0,0,127,149,149,149,149,0,0,0,0]
        self.assertTrue(self.passes(checks,baseline))
        self.assertEqual({c.get('sample',c.get('to_sample')) for c in checks},set(range(12))-{3,4})
        for sample in [0,1,2,5,6,7,8,9,10,11]:
            bad=baseline.copy();bad[sample]+=1
            self.assertFalse(self.passes(checks,bad),sample)
        self.assertNotIn('maximum',layouts['rf']['execution']['memory_probes'][0])
        self.assertEqual(layouts['rf']['execution']['memory_probes'][1:],other['execution']['memory_probes'][1:])
        self.assertEqual(layouts['valve'],other)
    def test_missing_expiry_probe_cannot_qualify_as_zero(self):
        topology,layouts=self.fixture();layouts['rf']['execution']['memory_probes'].pop(0)
        before=copy.deepcopy(layouts)
        with self.assertRaisesRegex(ValueError,'CAN expiry probe missing'):
            module.scope_can_ack_fault_expiry(topology,layouts)
        self.assertEqual(layouts,before)
    def test_connected_scenario_is_unchanged(self):
        topology,layouts=self.fixture();topology['can_ack_events']=[]
        before=copy.deepcopy((topology,layouts));module.scope_can_ack_fault_expiry(topology,layouts)
        self.assertEqual((topology,layouts),before)
    def test_incomplete_or_reversed_fault_cannot_remove_any_bound(self):
        for mode in ['no_restore','no_outage','reversed']:
            topology,layouts=self.fixture()
            if mode=='no_restore':topology['can_ack_events'].pop()
            elif mode=='no_outage':topology['can_ack_events'].pop(0)
            else:topology['can_ack_events'][1]['after_sample']=2
            before=copy.deepcopy(layouts)
            with self.assertRaises(ValueError):module.scope_can_ack_fault_expiry(topology,layouts)
            self.assertEqual(layouts,before)

if __name__=='__main__':unittest.main()
