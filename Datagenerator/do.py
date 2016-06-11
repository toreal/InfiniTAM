import os
import prman
def gen( val,fn,channel ):
    ri=prman.Ri()
    render='trans.rib'
    ri.Begin(render)
    ri.Display(fn, "tiff", channel)
    ri.Translate(0,0,5)
    ri.Rotate(-30,1,0,0)
    ri.Rotate(val,0,1,0)
    ri.End()
    


for i in range(5):
    cmd='prman -t:4  test.rib'
    fn='ttd_'+str(i)+'.tif'
    gen(i*10,fn ,"z")
    os.system(cmd)
    fn2='ttrgb_'+str(i)+'.tif'
    gen(i*10,fn2,"rgba")	
    os.system(cmd)
    print(i)