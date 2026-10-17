#!/usr/bin/env python3
"""
Tests for cbor_registry.py: each forbidden change to the registry must be caught, a changed default must be flagged
for review, and the checked-in registry must validate and match its generated sources.

    python3 -m unittest scripts/test_cbor_registry.py      (from the SDK root)
"""

import copy
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cbor_registry as cr  # noqa: E402


def registry():
    return cr.load(cr.read_text(cr.REGISTRY_PATH))


def did(reg, name):
    return next(d for d in reg['dids'] if d['name'] == name)


def field(reg, did_name, field_name):
    return next(f for f in did(reg, did_name)['fields'] if f['name'] == field_name)


def run_compare(base, head):
    rep = cr.Report()
    cr.validate(head, rep)
    cr.compare(base, head, rep)
    return rep


class CheckedInRegistry(unittest.TestCase):
    def test_validates(self):
        rep = cr.Report()
        cr.validate(registry(), rep)
        self.assertEqual(rep.errors, [])

    def test_generated_sources_are_current(self):
        reg = registry()
        for path, text in cr.generated_files(reg).items():
            with self.subTest(path=os.path.basename(path)):
                self.assertEqual(cr.read_text(path), text, 'run scripts/cbor_registry.py generate')

    def test_unchanged_registry_has_no_findings(self):
        reg = registry()
        rep = run_compare(reg, copy.deepcopy(reg))
        self.assertEqual((rep.errors, rep.reviews), ([], []))


class ForbiddenChanges(unittest.TestCase):
    def setUp(self):
        self.base = registry()
        self.head = copy.deepcopy(self.base)

    def assertError(self, text):
        rep = run_compare(self.base, self.head)
        self.assertTrue(any(text in e for e in rep.errors), f'expected an error containing {text!r}, got {rep.errors}')

    def test_renumbered_key(self):
        field(self.head, 'FLASH_CFG', 'ser0BaudRate')['key'] = 60
        self.assertError('renumbered')

    def test_reused_key(self):
        field(self.head, 'FLASH_CFG', 'ser0BaudRate')['name'] = 'somethingElse'
        self.assertError('renamed/reused')

    def test_retyped_key(self):
        field(self.head, 'FLASH_CFG', 'ser0BaudRate')['type'] = 'uint16'
        self.assertError('type changed')

    def test_changed_count(self):
        field(self.head, 'FLASH_CFG', 'insRotation')['count'] = 4
        self.assertError('count changed')

    def test_removed_key(self):
        d = did(self.head, 'FLASH_CFG')
        d['fields'] = [f for f in d['fields'] if f['name'] != 'debug']
        self.assertError('was removed')

    def test_removed_envelope_key(self):
        self.head['message'] = [e for e in self.head['message'] if e['name'] != 'MSG_REQ_ID']
        self.assertError('was removed')

    def test_revived_deprecated_key(self):
        field(self.base, 'FLASH_CFG', 'debug')['deprecated'] = True
        self.assertError('cannot be revived')

    def test_changed_enum_value(self):
        self.head['enums']['msg_status']['READ_ONLY'] = 9
        self.assertError('changed value')

    def test_key_outside_range(self):
        self.head['message'][0]['key'] = 0x9000
        self.assertError('outside the message range')

    def test_duplicate_key_in_scope(self):
        field(self.head, 'FLASH_CFG', 'debug')['key'] = 7
        self.assertError('used by both')

    def test_default_and_default_from(self):
        field(self.head, 'FLASH_CFG', 'debug')['default_from'] = 'firmware_logic'
        self.assertError('exactly one of default / default_from')

    def test_neither_default_nor_default_from(self):
        del field(self.head, 'FLASH_CFG', 'debug')['default']
        self.assertError('exactly one of default / default_from')

    def test_data_option_reused_across_levels(self):
        did(self.head, 'RMC_STORED')['indexed_fields']['element']['fields'][0]['key'] = 0x8000
        self.assertError('data option 0x8000 used by both')

    def test_unknown_subscription_option_key(self):
        self.head['schemas']['subscriptions']['indexed_fields']['element']['did_keyed_fields']['element']['option_keys'].append('SUB_NOPE')
        self.assertError('option_keys names unknown key SUB_NOPE')

    def test_changed_transport_did(self):
        self.head['transport']['isb_did'] = 130
        self.assertError('transport changed')

    def test_message_type_overlaps_a_flag(self):
        self.head['enums']['msg_type']['BIG'] = 0x80
        self.assertError('overlaps msg_type_flag')

    def test_message_type_flag_not_a_single_bit(self):
        self.head['enums']['msg_type_flag']['TWO_BITS'] = 0x60
        self.assertError('is not a single bit')

    def test_field_too_large_for_one_reply(self):
        field(self.head, 'CAN_CONFIG', 'can_transmit_address')['count'] = 1000
        self.assertError('cannot fit one reply')


class ReviewAndWarnings(unittest.TestCase):
    def test_changed_default_is_flagged_for_review(self):
        base = registry()
        head = copy.deepcopy(base)
        field(head, 'FLASH_CFG', 'ser0BaudRate')['default'] = 115200
        rep = run_compare(base, head)
        self.assertEqual(rep.errors, [])
        self.assertTrue(any('ser0BaudRate default changed' in r for r in rep.reviews), rep.reviews)

    def test_adding_a_field_is_allowed(self):
        base = registry()
        head = copy.deepcopy(base)
        did(head, 'FLASH_CFG')['fields'].append({'key': 50, 'name': 'newField', 'type': 'uint8', 'default': 0})
        rep = run_compare(base, head)
        self.assertEqual((rep.errors, rep.reviews), ([], []))

    def test_whole_did_too_large_is_a_warning(self):
        reg = registry()
        did(reg, 'CAN_CONFIG')['fields'].append(
            {'key': 4, 'name': 'bigArray', 'type': 'uint32', 'count': 200, 'default': 0})
        rep = cr.Report()
        cr.validate(reg, rep)
        self.assertEqual(rep.errors, [])
        self.assertTrue(any('whole-DID worst case' in w for w in rep.warnings), rep.warnings)


if __name__ == '__main__':
    unittest.main()
