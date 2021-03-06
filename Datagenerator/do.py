import os
import prman
def gen( val,fn,channel ):
    ri=prman.Ri()
    render='trans.rib'
    ri.Begin(render)
    ri.Display(fn, "tiff", channel)

    if channel == 'rgba':
        ri.Projection(ri.PERSPECTIVE ,{ri.FOV:53.28})
        ri.Quantize ('rgba',  255,0,255,0.5 )
        ri.Translate(0,0,300)		
    else:
#        ri.Translate(0,0,300)	
#        ri.Rotate(-0.003870486249233254,  0.51323804,  0.30595541,  0.80185909)		
        ri.Projection(ri.PERSPECTIVE ,{ri.FOV:66.95})
        ri.Translate(-25.7004433,-1.33896589,-3.75516105)
        ri.Translate(0,0,300)
		
    
    ri.Rotate(-30,1,0,0)
    ri.Rotate(val,0,1,0)
    ri.End()
    


for i in range(37):
    cmd='prman -t:4  test.rib'
    fn='00'+str(i)+'.tif'
    gen(i*10+45,fn ,"z")
    os.system(cmd)
    fn2='c00'+str(i)+'.tif'
    gen(i*10+45,fn2,"rgba")	
    os.system(cmd)
    print(i)