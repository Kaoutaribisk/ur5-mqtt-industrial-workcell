import json
import unittest
from workcell_backend import decode_command, CONTROL, LEGACY_COMMAND

class Commands(unittest.TestCase):
    def test_preserves_manual_format(self):
        result=decode_command(LEGACY_COMMAND,b'2:up:10')
        self.assertEqual((result['joint'],result['direction'],result['value']),(2,'up',10))
    def test_rejects_nonfinite_and_invalid_manual_commands(self):
        for command in (b'0:up:nan',b'6:up:10',b'0:other:10',b'0:up:-2'):
            with self.assertRaises(ValueError): decode_command(LEGACY_COMMAND,command)
    def test_rejects_bad_production_actions_and_speed(self):
        for value in (dict(action='delete'),dict(action='speed',value=0),dict(action='speed',value=True),dict(action='speed',value=float('inf'))):
            with self.assertRaises(ValueError): decode_command(CONTROL,json.dumps(value).encode())
    def test_accepts_stop_and_speed(self):
        self.assertEqual(decode_command(CONTROL,b'{"action":"stop","id":"abc"}')['id'],'abc')
        self.assertEqual(decode_command(CONTROL,b'{"action":"speed","value":8}')['value'],8)

if __name__=='__main__': unittest.main()
