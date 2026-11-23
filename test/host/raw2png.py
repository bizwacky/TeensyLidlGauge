# Turn the render harness's raw RGB565 frames into one PNG contact sheet:
#   python3 raw2png.py <out.png> <scene> [<scene> ...]      (K=2 for 2x zoom)
# Each scene is a row of the three panels. No dependencies beyond Python 3.
import sys, struct, zlib
W,H=428,142
def load(fn):
    d=open(fn,'rb').read()
    px=struct.unpack('<%dH'%(W*H), d)
    rows=[]
    for y in range(H):
        row=bytearray()
        for x in range(W):
            v=px[y*W+x]
            r=(v>>11)&31; g=(v>>5)&63; b=v&31
            row+=bytes(((r*255)//31,(g*255)//63,(b*255)//31))
        rows.append(bytes(row))
    return rows
def png(fn, rows, w):
    raw=b''.join(b'\x00'+r for r in rows)
    def chunk(t,d): return struct.pack('>I',len(d))+t+d+struct.pack('>I',zlib.crc32(t+d)&0xffffffff)
    open(fn,'wb').write(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,len(rows),8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw,9))+chunk(b'IEND',b''))
def scale(rows, k):
    out=[]
    for r in rows:
        rr=b''.join(r[i:i+3]*k for i in range(0,len(r),3))
        out+= [rr]*k
    return out
def sheet(tags, out, k=2, gap=6):
    # one row per tag: three pages side by side
    allrows=[]
    for tag in tags:
        pages=[scale(load('out_%s_%d.raw'%(tag,p)),k) for p in range(3)]
        g=bytes([40,40,40])*gap
        for y in range(H*k):
            allrows.append(pages[0][y]+g+pages[1][y]+g+pages[2][y])
        allrows += [bytes([40,40,40])*((W*k)*3+gap*2)]*gap
    png(out, allrows, (W*k)*3+gap*2)
if __name__=='__main__':
    sheet(sys.argv[2:], sys.argv[1], k=int(__import__('os').environ.get('K','2')))
