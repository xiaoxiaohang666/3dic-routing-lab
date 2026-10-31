"""E0 arithmetic/evidence limits must not become a gain gate."""
import math,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import analyze_a150_v11_diagnostics as d
class DiagnosticsTests(unittest.TestCase):
    def test_fixed_and_added_are_differences(self):
        fixed={'totals':{'new_fixed_only_overflow':10,'new_fixed_only_Q_0p1_proxy':1.,'added_pin_floor':3,'raw_fixed_track_union':2,'new_fixed_total':5}}
        out=d.summarize_resources({'overflow':12,'Q':4.,'C':100/math.sqrt(5)},fixed)
        self.assertEqual(out['route_added_overflow'],2)
        self.assertEqual(out['route_added_Q_0p1_proxy'],3.)
        self.assertIn('not Q(R)',out['added_Q_semantics'])
        self.assertIn('Not official',out['bound_conditions'])
    def test_noncausal_overlap(self):
        out=d.rejection_overlap(100,10,60,40,20)
        self.assertEqual(out['union_bounds'],[80,80])
        self.assertFalse(out['is_causal_attribution'])
        self.assertFalse(out['residual_is_rejection_count'])
        with self.assertRaises(ValueError): d.rejection_overlap(100,10,60,40,0)
    def test_reject_impossible_fixed_bound(self):
        with self.assertRaises(ValueError): d.added(3,5,'Q')
        with self.assertRaises(ValueError): d.number(float('nan'),'Q')
        with self.assertRaises(ValueError): d.ratio(5,4,'unique','population')
    def test_existing_output_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);out=root/'existing';out.mkdir();sentinel=out/'keep';sentinel.write_text('safe')
            with self.assertRaises(FileExistsError): d.write_outputs({'evidence_root':str(root/'evidence')},out)
            self.assertEqual(sentinel.read_text(),'safe')
    def test_evidence_tree_not_writable_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            with self.assertRaises(ValueError): d.write_outputs({'evidence_root':str(root)},root/'new')
if __name__=='__main__':unittest.main()
