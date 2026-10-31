import subprocess,sys,unittest
BINARY=sys.argv.pop(1)
class SiteCLI(unittest.TestCase):
 def run_cli(self,*args):return subprocess.run([BINARY,*args],capture_output=True,text=True)
 def test_help(self):self.assertIn('--objective-profile',self.run_cli('--help').stdout)
 def test_invalid(self):self.assertIn('invalid objective profile',self.run_cli('--objective-profile','bad').stderr)
 def test_explicit_gain_required(self):self.assertIn('explicit --thermal-gain',self.run_cli('--model','official','--objective-profile','site-20261029-scalars').stderr)
if __name__=='__main__':unittest.main()
