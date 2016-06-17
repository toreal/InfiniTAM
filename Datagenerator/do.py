import os
import prman
def gen( val,fn,channel ):
    ri=prman.Ri()
    render='trans.rib'
    ri.Begin(render)
    ri.Display(fn, "tiff", channel)
    ri.Translate(0,-10,300)
    ri.Rotate(-30,1,0,0)
    ri.Rotate(val,0,1,0)
    ri.End()
    


for i in range(37):
    cmd='prman -t:4  test.rib'
    fn='00'+str(i)+'.tif'
    gen(i*10,fn ,"z")
    os.system(cmd)
    fn2='c00'+str(i)+'.tif'
    gen(i*10,fn2,"rgba")	
    os.system(cmd)
    print(i)