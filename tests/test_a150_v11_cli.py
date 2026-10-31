"""Reject invalid experimental parameters and incompatible scoring profiles."""
import subprocess, sys, unittest
from pathlib import Path
ROUTER=Path(sys.argv.pop(1)).resolve()
class TreeMoveCliTests(unittest.TestCase):
    def invoke(self,*args):
        return subprocess.run([str(ROUTER),*args],capture_output=True,text=True,timeout=10)
    def test_bounds(self):
        for flag, values in {
            '--tsv-tree-move':['2','-1','nan'],
            '--tsv-tree-move-seconds':['0','-1','nan','inf','1.01','0.02x'],
            '--tsv-tree-move-expansions':['0','-1','1000001','1.5'],
        }.items():
            for value in values:
                with self.subTest(flag=flag,value=value):
                    result=self.invoke(flag,value)
                    self.assertNotEqual(result.returncode,0)
                    self.assertNotIn('required: --def',result.stderr)
    def test_requires_new_contract(self):
        result=self.invoke('--tsv-tree-move','1')
        self.assertNotEqual(result.returncode,0)
        self.assertIn('TSV tree move requires',result.stderr)
    def test_default_is_off_and_cli_exposed(self):
        result=self.invoke('--help')
        self.assertEqual(result.returncode,0)
        self.assertIn('--tsv-tree-move',result.stdout)
        result=self.invoke('--tsv-tree-move','0','--tsv-tree-move-seconds','0.02','--tsv-tree-move-expansions','4096')
        self.assertNotIn('unknown option',result.stderr)
        self.assertNotIn('TSV tree move requires',result.stderr)
        self.assertNotEqual(result.returncode,0)
if __name__=='__main__':unittest.main()
