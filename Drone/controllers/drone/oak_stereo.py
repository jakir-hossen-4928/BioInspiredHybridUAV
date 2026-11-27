"""Rectified grayscale stereo edge matching gated by a red obstacle in RGB."""
import math

def objects(rgb,left,right, mount_x):
    width,height=left.getWidth(),left.getHeight()
    rgb_width,rgb_height=rgb.getWidth(),rgb.getHeight()
    image=rgb.getImage()
    views=[cam.getImage() for cam in (left,right)]
    if not image or not all(views):
        return []
    f=width/(2*math.tan(rgb.getFov()/2))
    row=height//2
    def red(x):
        i=4*((rgb_height//2)*rgb_width+x*rgb_width//width)
        b,g,r=image[i:i+3]
        return r>65 and r>2*g and r>2*b
    xs=[x for x in range(width) if red(x)]
    groups=[]
    for x in xs:
        if not groups or x>groups[-1][-1]+1:
            groups.append([])
        groups[-1].append(x)
    def gray(view,x,y):
        i=4*(y*width+x)
        b,g,r=view[i:i+3]
        return (r+2*g+b)//4
    hits=[]
    for group in groups:
        if len(group)<6: continue
        edge=group[0]
        if edge<35 or edge>width-35: continue
        # Locate the left-camera silhouette, then match its intensity patch on the right.
        candidates=range(edge,edge+16)
        u=max(candidates,key=lambda x:abs(gray(views[0],x+1,row)-gray(views[0],x-1,row)))
        def cost(d):
            return sum(abs(gray(views[0],u+i,row+j)-gray(views[1],u+i-d,row+j))
                       for i in range(-3,4) for j in (-3,0,3))
        scores=sorted((cost(d),d) for d in range(1,31))
        best,disparity=scores[0]
        if scores[1][0]-best<12: continue
        z=f*.075/disparity
        lateral=(width/2-(group[0]+group[-1])/2)*z/f
        hits.append((z+mount_x,lateral,len(group)*z/f,disparity))
    return hits
