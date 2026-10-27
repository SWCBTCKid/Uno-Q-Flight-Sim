#!/usr/bin/env python3
"""Dependency-free ELF64 ABI/dependency inspector."""
import argparse, json, struct, sys
from pathlib import Path
PT_LOAD, PT_DYNAMIC, PT_INTERP = 1, 2, 3
DT_NULL, DT_NEEDED, DT_STRTAB, DT_RPATH, DT_RUNPATH = 0, 1, 5, 15, 29
def inspect(path):
    b=path.read_bytes()
    if len(b)<64 or b[:4]!=b'\x7fELF': raise ValueError('not an ELF file')
    if b[4]!=2 or b[5]!=1: raise ValueError('requires ELF64 little-endian')
    typ,machine=struct.unpack_from('<HH',b,16); phoff=struct.unpack_from('<Q',b,32)[0]
    phentsize,phnum=struct.unpack_from('<HH',b,54); segs=[]
    for i in range(phnum):
        o=phoff+i*phentsize
        if o+56>len(b): raise ValueError('truncated program header')
        ptype,_,off,va,_,filesz,memsz,_=struct.unpack_from('<IIQQQQQQ',b,o)
        segs.append((ptype,off,va,filesz,memsz))
    def cstr(off):
        end=b.find(b'\0',off)
        if off<0 or off>=len(b) or end<0: raise ValueError('invalid string offset')
        return b[off:end].decode('utf-8','replace')
    interp=None; dyn=[]
    for ptype,off,_,filesz,_ in segs:
        if ptype==PT_INTERP: interp=cstr(off)
        if ptype==PT_DYNAMIC:
            for q in range(off,off+filesz,16):
                tag,val=struct.unpack_from('<qQ',b,q); dyn.append((tag,val))
                if tag==DT_NULL: break
    def vaoff(addr):
        for ptype,off,va,filesz,_ in segs:
            if ptype==PT_LOAD and va<=addr<va+filesz:return off+addr-va
        raise ValueError('string table not file-backed')
    strva=next((v for t,v in dyn if t==DT_STRTAB),None); needed=[]; rpaths=[]
    if strva is not None:
        so=vaoff(strva); needed=[cstr(so+v) for t,v in dyn if t==DT_NEEDED]; rpaths=[cstr(so+v) for t,v in dyn if t in (DT_RPATH,DT_RUNPATH)]
    return {'path':str(path.resolve()),'elf_class':64,'endianness':'little','type':typ,'machine':machine,'machine_name':'AArch64' if machine==183 else f'EM_{machine}','interpreter':interp,'needed':needed,'rpath_runpath':rpaths,'size_bytes':len(b)}
def main():
    ap=argparse.ArgumentParser();ap.add_argument('elf',type=Path);ap.add_argument('--require-aarch64',action='store_true');ap.add_argument('--output',type=Path);a=ap.parse_args()
    try:r=inspect(a.elf)
    except Exception as e:print(f'ELF inspection failed: {e}',file=sys.stderr);return 2
    errors=[]
    if a.require_aarch64 and r['machine']!=183:errors.append('ELF machine is not AArch64')
    if r['rpath_runpath']:errors.append('RPATH/RUNPATH is forbidden')
    forbidden={'libstdc++.so.6','libc++.so.1','libgcc_s.so.1'}
    unexpected=sorted(forbidden.intersection(r['needed']))
    if unexpected:errors.append('forbidden C++ runtime dependencies: '+', '.join(unexpected))
    r['valid']=not errors;r['errors']=errors;text=json.dumps(r,indent=2,sort_keys=True)
    if a.output:a.output.write_text(text+'\n',encoding='utf-8')
    print(text);return 0 if not errors else 3
if __name__=='__main__':raise SystemExit(main())
