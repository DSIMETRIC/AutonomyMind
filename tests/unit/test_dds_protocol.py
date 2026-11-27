import unittest
from autonomymind_runtime.dds_protocol import envelope,encode,decode,Receiver,PeerDirectory
class ProtocolTests(unittest.TestCase):
    def test_roundtrip_duplicate_expiry(self):
        m=decode(encode(envelope("a","b","data",{},10,2)))
        r=Receiver("b",{"data"})
        self.assertEqual(r.check(m,10),"accepted")
        self.assertEqual(r.check(m,11),"duplicate")
        self.assertEqual(r.check(m,12),"expired")
    def test_discovery_expiry_and_replay(self):
        d=PeerDirectory(3)
        m=envelope("a","*","heartbeat",{"capabilities":["allocate"]},10,3)
        self.assertTrue(d.observe(m,10,100))
        self.assertFalse(d.observe(m,11,101))
        self.assertEqual(d.discover("allocate",102),["a"])
        self.assertEqual(d.discover("allocate",103),[])
    def test_contract(self):
        m=envelope("a","b","bad",{},10,2)
        self.assertEqual(Receiver("b",{"data"}).check(m,10),"contract_mismatch")
