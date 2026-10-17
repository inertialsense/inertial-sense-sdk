#!/usr/bin/env python3
"""
CBOR key registry tool: validates src/cbor/registry.yaml, checks it against a previous version, and generates the
C sources derived from it.

    cbor_registry.py generate [--check]          write (or, with --check, verify) the generated sources
    cbor_registry.py check [--base REF | --base-file PATH] [--registry PATH]
                                                 validate the registry; with a base, also check changes against it

Exit status: 0 = ok (warnings and review flags are printed), 1 = errors, 2 = usage.
Design: is-common/docs_internal/cbor/CBOR_PROTOCOL_DESIGN.md (sections 3, 4, 7, 10).
"""

import argparse
import os
import re
import subprocess
import sys

import yaml

SDK_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
REGISTRY_REL = 'src/cbor/registry.yaml'
REGISTRY_PATH = os.path.join(SDK_DIR, REGISTRY_REL)
GEN_DIR = os.path.join(SDK_DIR, 'src', 'cbor')

SCALAR_TYPES = {
    # registry type: (C type, FCTYPE scalar, FCTYPE array, worst-case CBOR bytes)
    'uint8':   ('uint8_t',  'FCTYPE_UINT8',  'FCTYPE_UINT8_ARRAY',  2),
    'uint16':  ('uint16_t', 'FCTYPE_UINT16', 'FCTYPE_UINT16_ARRAY', 3),
    'uint32':  ('uint32_t', 'FCTYPE_UINT32', 'FCTYPE_UINT32_ARRAY', 5),
    'uint64':  ('uint64_t', 'FCTYPE_UINT64', None,                  9),
    'int8':    ('int8_t',   None, None, 2),
    'int16':   ('int16_t',  None, None, 3),
    'int32':   ('int32_t',  None, None, 5),
    'int64':   ('int64_t',  None, None, 9),
    'float32': ('float',    'FCTYPE_FLOAT',  'FCTYPE_FLOAT_ARRAY',  5),
    'float64': ('double',   'FCTYPE_DOUBLE', 'FCTYPE_DOUBLE_ARRAY', 9),
    'bool':    ('uint8_t',  None, None, 1),
}
COMPOUND_TYPES = {'array', 'map', 'key_paths', 'text', 'bytes'}
DEFAULT_SOURCES = {'firmware_logic', 'protocol_rule'}

# Worst-case bytes of a reply envelope (map header, MSG_TYPE, MSG_SRC_DEV_ID, MSG_DST_DEV_FILTER, MSG_REQ_ID,
# MSG_TIME_GPS_NS, MSG_ROUTE) plus the DID key and the DID's map header.
REPLY_OVERHEAD = 3 + (3 + 2) + (3 + 9) + (3 + 9) + (3 + 3) + (3 + 9) + (3 + 1 + 4 * 2) + 3 + 3


class Report:
    def __init__(self):
        self.errors, self.warnings, self.reviews = [], [], []

    def error(self, msg): self.errors.append(msg)
    def warn(self, msg): self.warnings.append(msg)
    def review(self, msg): self.reviews.append(msg)

    def print(self):
        for tag, items in (('ERROR', self.errors), ('REVIEW', self.reviews), ('WARNING', self.warnings)):
            for m in items:
                print(f'{tag}: {m}')


# ------------------------------------------------------------------------------------------------ loading

def read_text(path):
    with open(path) as fh:
        return fh.read()


def load(text):
    reg = yaml.safe_load(text)
    if not isinstance(reg, dict):
        raise ValueError('registry is not a mapping')
    for did in reg.get('dids', []):
        name = did.get('schema')
        if name:
            for k, v in reg['schemas'][name].items():
                did.setdefault(k, v)
    return reg


def load_base(ref, registry_rel=REGISTRY_REL):
    """Returns the registry at git ref, or None if the file did not exist there."""
    try:
        text = subprocess.check_output(['git', '-C', SDK_DIR, 'show', f'{ref}:{registry_rel}'],
                                       stderr=subprocess.DEVNULL, text=True)
    except subprocess.CalledProcessError:
        return None
    return load(text)


def resolve_count(reg, count):
    if count is None:
        return 1
    if isinstance(count, int):
        return count
    return int(reg.get('constants', {})[count])


def resolve_size(reg, value):
    return value if isinstance(value, int) else int(reg.get('constants', {})[value])


# ------------------------------------------------------------------------------------------------ worst-case size

def worst_size(reg, f):
    """Worst-case CBOR bytes of a field's value (not its key)."""
    t = f.get('type')
    if t in SCALAR_TYPES:
        n = resolve_count(reg, f.get('count'))
        elem = SCALAR_TYPES[t][3]
        return elem if f.get('count') is None else 3 + n * elem
    if t == 'array':
        n = resolve_count(reg, f.get('max_count', f.get('count')))
        elem = f['element']
        es = SCALAR_TYPES[elem][3] if isinstance(elem, str) else worst_size(reg, elem)
        return 3 + n * es
    if t == 'map':
        total = 3 + sum(3 + worst_size(reg, sub) for sub in f.get('fields', []))
        dk = f.get('did_keyed_fields')
        if dk:
            total += resolve_count(reg, dk['max_count']) * (3 + entry_worst_size(reg, dk['element']))
        return total
    if t in ('text', 'bytes'):
        return 3 + resolve_count(reg, f.get('max_count', f.get('count')))
    if t == 'key_paths':
        return 3 + 16 * 9
    return 0


def entry_worst_size(reg, elem):
    """Worst-case bytes of a map whose keys are named option keys (e.g. a subscription entry)."""
    by_name = {e['name']: e for e in reg.get('message', [])}
    for entries in (reg.get('message_type') or {}).values():
        by_name.update({e['name']: e for e in entries})
    return 3 + sum(3 + worst_size(reg, by_name[k]) for k in elem.get('option_keys', []) if k in by_name) + \
        sum(3 + worst_size(reg, sub) for sub in elem.get('fields', []))


def did_worst_size(reg, did):
    total = 3 + sum(3 + worst_size(reg, f) for f in did.get('fields') or [])
    ix = did.get('indexed_fields')
    if ix:
        total += resolve_count(reg, ix['max_count']) * (3 + worst_size(reg, ix['element']))
    return total


# ------------------------------------------------------------------------------------------------ validation

def did_range(reg, did):
    if 'legacy' in did:
        return 'legacy_did'
    if did.get('nmea'):
        return 'nmea_did'
    return 'protocol_did' if did['key'] >= reg['ranges']['protocol_did'][0] else 'application_did'


def field_range(key):
    return 'field' if key < 0x8000 else 'data_option'


def walk_fields(scope, fields):
    """Yields (scope, key, entry, range_name) for fields and anything nested inside them."""
    for f in fields or []:
        yield (scope, f['key'], f, field_range(f['key']))
        yield from walk_fields(f'{scope}.{f["name"]}', f.get('fields'))
        elem = f.get('element')
        if isinstance(elem, dict):
            yield from walk_fields(f'{scope}.{f["name"]}[]', elem.get('fields'))


def did_children(did):
    """Yields (scope suffix, fields) for each field set a DID defines, directly or through its element maps."""
    yield ('', did.get('fields'))
    ix = did.get('indexed_fields')
    if ix:
        elem = ix['element']
        yield ('[]', elem.get('fields'))
        dk = elem.get('did_keyed_fields')
        if dk:
            yield ('[][did]', dk['element'].get('fields'))
    dk = did.get('did_keyed_fields')
    if dk:
        yield ('[did]', dk['element'].get('fields'))


def iter_keyed(reg):
    """Yields (scope, key, entry, range_name) for every key the registry defines."""
    for e in reg.get('message', []):
        yield ('message', e['key'], e, 'message')
    for mtype, entries in (reg.get('message_type') or {}).items():
        for e in entries:
            yield (f'message_type.{mtype}', e['key'], e, 'message_type')
    for e in reg.get('protocol', []) or []:
        yield ('protocol', e['key'], e, 'protocol')
    for did in reg.get('dids', []):
        yield ('root', did['key'], did, did_range(reg, did))
        for suffix, fields in did_children(did):
            yield from walk_fields(f'did.{did["name"]}{suffix}', fields)


def option_key_names(reg):
    names = {e['name'] for e in reg.get('message', [])}
    for entries in (reg.get('message_type') or {}).values():
        names |= {e['name'] for e in entries}
    return names


def check_defaults(scope, e, rep):
    if e.get('type') == 'map' and 'fields' in e:
        return
    has_value, has_source = 'default' in e, 'default_from' in e
    if has_value == has_source:
        rep.error(f'{scope}: {e.get("name")} must have exactly one of default / default_from')
    if has_source and e['default_from'] not in DEFAULT_SOURCES:
        rep.error(f'{scope}: {e.get("name")} default_from must be one of {sorted(DEFAULT_SOURCES)}')


def validate(reg, rep):
    ranges = reg.get('ranges', {})
    seen = {}
    for scope, key, e, rng in iter_keyed(reg):
        name = e.get('name')
        lo, hi = ranges[rng]
        if not (lo <= key <= hi):
            rep.error(f'{scope}: {name} key 0x{key:04X} is outside the {rng} range 0x{lo:04X}-0x{hi:04X}')
        if (scope, key) in seen:
            rep.error(f'{scope}: key 0x{key:04X} used by both {seen[(scope, key)]} and {name}')
        seen[(scope, key)] = name
        if scope != 'root':
            t = e.get('type')
            if t not in SCALAR_TYPES and t not in COMPOUND_TYPES:
                rep.error(f'{scope}: {name} has unknown type {t!r}')
            if 'enum' in e and e['enum'] not in reg.get('enums', {}):
                rep.error(f'{scope}: {name} refers to unknown enum {e["enum"]!r}')
            check_defaults(scope, e, rep)

    names = {}
    for scope, key, e, rng in iter_keyed(reg):
        n = e.get('name')
        if (scope, n) in names and names[(scope, n)] != key:
            rep.error(f'{scope}: name {n} used for two keys')
        names[(scope, n)] = key

    for did in reg.get('dids', []):
        seen_opts = {}
        for scope, key, e, rng in walk_all(did):
            if rng == 'data_option':
                if key in seen_opts:
                    rep.error(f'{did["name"]}: data option 0x{key:04X} used by both {seen_opts[key]} and {e["name"]}')
                seen_opts[key] = e['name']

    known = option_key_names(reg)
    for did in reg.get('dids', []):
        for dk in (did.get('did_keyed_fields'), (did.get('indexed_fields') or {}).get('element', {}).get('did_keyed_fields')):
            for k in (dk or {}).get('element', {}).get('option_keys', []):
                if k not in known:
                    rep.error(f'{did["name"]}: option_keys names unknown key {k}')

    devices = reg.get('devices', {})
    for did in reg.get('dids', []):
        if did.get('reserved'):
            continue
        for dev in did.get('served_by', []):
            if dev not in devices:
                rep.error(f'{did["name"]}: served_by names unknown device {dev}')
        bufs = [resolve_size(reg, devices[d]['reply_buffer']) for d in did.get('served_by', []) if d in devices]
        if not bufs:
            continue
        limit = min(bufs) - REPLY_OVERHEAD
        for f in did.get('fields', []):
            ws = 3 + worst_size(reg, f)
            if ws > limit:
                rep.error(f'{did["name"]}.{f["name"]}: worst case {ws} B cannot fit one reply ({limit} B available)')
        whole = did_worst_size(reg, did)
        if whole > limit:
            rep.warn(f'{did["name"]}: whole-DID worst case {whole} B exceeds one reply ({limit} B available); '
                     f'hosts must request it in parts')


def compare(base, head, rep):
    """Checks head against base: no key renumbered, reused, retyped or removed; changed defaults flagged."""
    def index(reg):
        out = {}
        for scope, key, e, _ in iter_keyed(reg):
            out[(scope, key)] = e
        return out

    if base.get('transport') != head.get('transport'):
        rep.error(f'transport changed: {base.get("transport")} -> {head.get("transport")}')
    b, h = index(base), index(head)
    for (scope, key), be in b.items():
        he = h.get((scope, key))
        name = be.get('name')
        if he is None:
            rep.error(f'{scope}: {name} (key 0x{key:04X}) was removed; mark it deprecated instead')
            continue
        if he.get('name') != name:
            rep.error(f'{scope}: key 0x{key:04X} renamed/reused: {name} -> {he.get("name")}')
        if scope != 'root':
            for attr in ('type', 'count', 'element', 'enum'):
                if be.get(attr) != he.get(attr):
                    rep.error(f'{scope}: {name} {attr} changed: {be.get(attr)!r} -> {he.get(attr)!r}')
            if (be.get('default'), be.get('default_from')) != (he.get('default'), he.get('default_from')):
                rep.review(f'{scope}: {name} default changed: '
                           f'{be.get("default", be.get("default_from"))!r} -> {he.get("default", he.get("default_from"))!r}')
        if be.get('deprecated') and not he.get('deprecated'):
            rep.error(f'{scope}: {name} was deprecated and cannot be revived')
    base_names = {(s, e.get('name')): k for (s, k), e in b.items()}
    for (scope, key), he in h.items():
        if (scope, key) not in b and (scope, he.get('name')) in base_names:
            rep.error(f'{scope}: {he.get("name")} renumbered: 0x{base_names[(scope, he.get("name"))]:04X} -> 0x{key:04X}')
    for name, vals in (base.get('enums') or {}).items():
        hv = (head.get('enums') or {}).get(name, {})
        for sym, v in vals.items():
            if sym not in hv:
                rep.error(f'enum {name}: {sym} removed')
            elif hv[sym] != v:
                rep.error(f'enum {name}: {sym} changed value {v} -> {hv[sym]}')


# ------------------------------------------------------------------------------------------------ generation

def walk_all(did):
    for suffix, fields in did_children(did):
        yield from walk_fields(suffix, fields)


HEADER = '/* Generated from src/cbor/registry.yaml by scripts/cbor_registry.py -- do not edit. */\n'


def c_ident(s):
    return re.sub(r'[^A-Za-z0-9]', '_', s).upper()


def c_value(v):
    if isinstance(v, bool):
        return '1' if v else '0'
    if isinstance(v, float):
        return repr(v) + 'f' if abs(v) < 3.4e38 else repr(v)
    return str(v)


def legacy_dids(reg):
    return [d for d in reg.get('dids', []) if 'legacy' in d and not d.get('reserved')]


def gen_header(reg):
    out = [HEADER, '#ifndef CBOR_REGISTRY_H', '#define CBOR_REGISTRY_H', '',
           '#include <stdint.h>', '#include "cbor/flash_cfg_cbor.h"', '',
           '#ifdef __cplusplus', 'extern "C" {', '#endif', '']
    out.append(f'#define CBOR_REGISTRY_VERSION  {reg["registry_version"]}')
    out.append(f'#define CBOR_ISB_DID           {reg["transport"]["isb_did"]}   /* DID_CBOR */')
    out += ['', '/* Root DID keys */']
    for d in reg.get('dids', []):
        out.append(f'#define CBOR_DID_{c_ident(d["name"])}  0x{d["key"]:04X}')
    out += ['', '/* General message keys */']
    for e in reg.get('message', []):
        out.append(f'#define CBOR_{e["name"]}  0x{e["key"]:04X}')
    out += ['', '/* Message-type option keys */']
    for _, entries in (reg.get('message_type') or {}).items():
        for e in entries:
            out.append(f'#define CBOR_{e["name"]}  0x{e["key"]:04X}')
    for name, vals in (reg.get('enums') or {}).items():
        out += ['', 'typedef enum', '{']
        for sym, v in vals.items():
            out.append(f'    CBOR_{c_ident(name)}_{sym} = {v},')
        out.append(f'}} eCbor{"".join(p.capitalize() for p in name.split("_"))};')
    for d in reg.get('dids', []):
        entries = [(scope, f) for scope, _, f, _ in walk_all(d)]
        if not entries:
            continue
        out += ['', f'/* {d["name"]} field keys */']
        for _, f in entries:
            out.append(f'#define CBOR_{c_ident(d["name"])}__{c_ident(f["name"])}  0x{f["key"]:04X}' if f['key'] >= 0x8000
                       else f'#define CBOR_{c_ident(d["name"])}__{c_ident(f["name"])}  {f["key"]}')
    out += ['', '/* Field tables for legacy DIDs (bound to their C structs) */']
    for d in legacy_dids(reg):
        n = c_ident(d['name'])
        out.append(f'extern const flash_cfg_field_desc_t g_cborFields_{n}[];')
        out.append(f'extern const uint16_t g_cborFieldsCount_{n};')
    out += ['', '/* Field names (SDK only: cbor_registry_names.c) */',
            'const char *cbor_registry_field_name(uint16_t did, uint16_t key);',
            'int cbor_registry_field_key(uint16_t did, const char *name);', '',
            '/* Registry defaults (cbor_registry_defaults.c): the struct holds every fixed default; fixed[i] is 1 when',
            '   field i of g_cborFields_<DID> has a fixed default and 0 when the firmware computes it. */']
    for d in legacy_dids(reg):
        if not d.get('stored'):
            continue
        n = c_ident(d['name'])
        out.append(f'extern const {d["legacy"]["struct"]} g_cborDefaults_{n};')
        out.append(f'extern const uint8_t g_cborDefaultFixed_{n}[];')
    out += ['', '#ifdef __cplusplus', '}', '#endif', '', '#endif /* CBOR_REGISTRY_H */', '']
    return '\n'.join(out)


def gen_tables(reg):
    out = [HEADER, '#include <stddef.h>', '#include "data_sets.h"', '#include "ISComm.h"', '#include "cbor/cbor_registry.h"', '']
    out.append(f'_Static_assert(DID_CBOR == {reg["transport"]["isb_did"]}, "registry.yaml transport.isb_did does not match DID_CBOR");')
    for cname, val in (reg.get('constants') or {}).items():
        out.append(f'_Static_assert(({cname}) == {val}, "registry.yaml constant {cname} does not match the C definition");')
    out.append('')
    for d in legacy_dids(reg):
        n, st = c_ident(d['name']), d['legacy']['struct']
        out.append(f'const flash_cfg_field_desc_t g_cborFields_{n}[] = {{')
        for f in d['fields']:
            ctype, fs, fa, _ = SCALAR_TYPES[f['type']]
            cnt = f.get('count')
            member = f.get('member', f['name'])
            ft = fa if cnt is not None else fs
            out.append(f'    {{ {f["key"]}, {ft}, {cnt if cnt is not None else 1}, offsetof({st}, {member}) }},')
        out += ['};', f'const uint16_t g_cborFieldsCount_{n} = sizeof(g_cborFields_{n}) / sizeof(g_cborFields_{n}[0]);', '']
        out.append(f'/* {d["name"]}: each entry matches its {st} member in type and size */')
        for f in d['fields']:
            ctype = SCALAR_TYPES[f['type']][0]
            member = f.get('member', f['name'])
            cnt = f.get('count')
            expr = f'(({st} *)0)->{member}' + ('[0]' if cnt is not None else '')
            out.append(f'_Static_assert(_Generic({expr}, {ctype}: 1, default: 0), "{d["name"]}.{f["name"]} type");')
            if cnt is not None:
                out.append(f'_Static_assert(sizeof((({st} *)0)->{member}) == sizeof({ctype}) * ({cnt}), "{d["name"]}.{f["name"]} count");')
        out.append('')
    return '\n'.join(out)


def gen_names(reg):
    out = [HEADER, '#include <string.h>', '#include "cbor/cbor_registry.h"', '',
           'typedef struct { uint16_t did; uint16_t key; const char *name; } cbor_registry_name_t;', '',
           'static const cbor_registry_name_t s_names[] = {']
    for d in reg.get('dids', []):
        for _, _, f, _ in walk_all(d):
            out.append(f'    {{ 0x{d["key"]:04X}, 0x{f["key"]:04X}, "{f["name"]}" }},')
    out += ['};', '',
            'const char *cbor_registry_field_name(uint16_t did, uint16_t key)', '{',
            '    for (size_t i = 0; i < sizeof(s_names) / sizeof(s_names[0]); i++)',
            '        if (s_names[i].did == did && s_names[i].key == key)',
            '            return s_names[i].name;',
            '    return NULL;', '}', '',
            'int cbor_registry_field_key(uint16_t did, const char *name)', '{',
            '    for (size_t i = 0; i < sizeof(s_names) / sizeof(s_names[0]); i++)',
            '        if (s_names[i].did == did && strcmp(s_names[i].name, name) == 0)',
            '            return s_names[i].key;',
            '    return -1;', '}', '']
    return '\n'.join(out)


def gen_defaults(reg):
    out = [HEADER, '#include "data_sets.h"', '#include "cbor/cbor_registry.h"', '']
    for d in legacy_dids(reg):
        if not d.get('stored'):
            continue
        n, st = c_ident(d['name']), d['legacy']['struct']
        out.append(f'const {st} g_cborDefaults_{n} = {{')
        for f in d['fields']:
            if 'default' not in f:
                continue
            member = f.get('member', f['name'])
            v = f['default']
            cnt = f.get('count')
            if cnt is None:
                if isinstance(v, str):
                    v = reg['enums'][f['enum']][v] if 'enum' in f else v
                out.append(f'    .{member} = {c_value(v)},')
            elif isinstance(v, list):
                out.append(f'    .{member} = {{ {", ".join(c_value(x) for x in v)} }},')
            elif v != 0:
                n_el = resolve_count(reg, cnt)
                out.append(f'    .{member} = {{ {", ".join([c_value(v)] * n_el)} }},')
        out += ['};', f'const uint8_t g_cborDefaultFixed_{n}[] = {{']
        out.append('    ' + ', '.join('1' if 'default' in f else '0' for f in d['fields']) + ',')
        out += ['};', '']
    return '\n'.join(out)


def generated_files(reg):
    return {
        os.path.join(GEN_DIR, 'cbor_registry.h'): gen_header(reg),
        os.path.join(GEN_DIR, 'cbor_registry.c'): gen_tables(reg),
        os.path.join(GEN_DIR, 'cbor_registry_names.c'): gen_names(reg),
        os.path.join(GEN_DIR, 'cbor_registry_defaults.c'): gen_defaults(reg),
    }


# ------------------------------------------------------------------------------------------------ main

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('generate')
    g.add_argument('--check', action='store_true', help='fail if the generated files are out of date')
    c = sub.add_parser('check')
    c.add_argument('--registry', default=REGISTRY_PATH)
    c.add_argument('--base', help='git ref to compare against (e.g. the merge base)')
    c.add_argument('--base-file', help='registry file to compare against')
    args = ap.parse_args(argv)

    if args.cmd == 'generate':
        reg = load(read_text(REGISTRY_PATH))
        rep = Report()
        validate(reg, rep)
        rep.print()
        if rep.errors:
            return 1
        stale = []
        for path, text in generated_files(reg).items():
            current = read_text(path) if os.path.exists(path) else None
            if current != text:
                stale.append(os.path.relpath(path, SDK_DIR))
                if not args.check:
                    with open(path, 'w') as fh:
                        fh.write(text)
        if args.check and stale:
            print('ERROR: generated files are out of date; run scripts/cbor_registry.py generate: ' + ', '.join(stale))
            return 1
        return 0

    reg = load(read_text(args.registry))
    rep = Report()
    validate(reg, rep)
    base = None
    if args.base_file:
        base = load(read_text(args.base_file))
    elif args.base:
        base = load_base(args.base)
        if base is None:
            print(f'note: no registry at {args.base}; nothing to compare')
    if base is not None:
        compare(base, reg, rep)
    rep.print()
    print(f'{len(rep.errors)} error(s), {len(rep.reviews)} review flag(s), {len(rep.warnings)} warning(s)')
    return 1 if rep.errors else 0


if __name__ == '__main__':
    sys.exit(main())
