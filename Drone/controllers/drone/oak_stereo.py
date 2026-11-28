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
        if group[0]<35 or group[-1]>width-35: continue
        # Match the whole silhouette, not one edge, and every column: sampling
        # every other column skips the last one, so a flat-coloured box ties
        # between two disparities and the matcher silently picks the wrong one.
        def cost(d):
            return sum(abs(gray(views[0],x,row+j)-gray(views[1],x-d,row+j))
                       for x in group for j in (-3,0,3))
        costs={d:cost(d) for d in range(1,31)}
        disparity=min(costs,key=costs.get)
        best=costs[disparity]
        if min(c for d,c in costs.items() if d!=disparity)-best<12: continue
        before,after=costs.get(disparity-1),costs.get(disparity+1)
        if before is not None and after is not None:
            curvature=before-2*best+after
            if curvature>0: disparity+=.5*(before-after)/curvature
        z=f*.075/disparity
        lateral=(width/2-(group[0]+group[-1])/2)*z/f
        hits.append((z+mount_x,lateral,len(group)*z/f,disparity))
    return hits
