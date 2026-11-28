"""Collision geometry adapted from the local Drone-muqadder patrol.
Uses world pose of each solid and local transforms in its boundingObject.
The caller can choose which altitude band to include in its plan.
"""
import math
from itertools import product

IDENTITY=(1.,0.,0.,0.,1.,0.,0.,0.,1.)

def field(node,name):
    try:
        f=node.getField(name)
        if f is None and hasattr(node,'getBaseNodeField'):f=node.getBaseNodeField(name)
        return f
    except (AttributeError,KeyError,RuntimeError,TypeError):return None

def value(node,name,method,default):
    f=field(node,name)
    return getattr(f,method)() if f is not None else default

def typ(node):
    return node.getBaseTypeName() if hasattr(node,'getBaseTypeName') else ''

def mv(m,v):return tuple(sum(m[i*3+j]*v[j] for j in range(3)) for i in range(3))
def mm(a,b):return tuple(sum(a[i*3+k]*b[k*3+j] for k in range(3)) for i in range(3) for j in range(3))
def add(a,b):return tuple(x+y for x,y in zip(a,b))

def rotation(axis):
    x,y,z,t=axis;n=math.sqrt(x*x+y*y+z*z)
    if n<1e-12:return IDENTITY
    x,y,z=x/n,y/n,z/n;c,s=math.cos(t),math.sin(t);d=1-c
    return (c+x*x*d,x*y*d-z*s,x*z*d+y*s,y*x*d+z*s,c+y*y*d,y*z*d-x*s,z*x*d-y*s,z*y*d+x*s,c+z*z*d)

def local_bounds(node):
    """Return conservative local corners for supported primitives or groups."""
    points=[]
    def visit(n,origin,m):
        k=typ(n)
        if k in ('Pose','Transform','Group'):
            origin=add(origin,mv(m,value(n,'translation','getSFVec3f',(0,0,0))))
            r=rotation(value(n,'rotation','getSFRotation',(0,0,1,0)))
            scale=value(n,'scale','getSFVec3f',(1,1,1))
            m=mm(m,tuple(r[i]*scale[i%3] for i in range(9)))
            children=field(n,'children')
            if children:
                for i in range(children.getCount()):visit(children.getMFNode(i),origin,m)
            return
        if k=='Shape':
            g=value(n,'geometry','getSFNode',None)
            if g is not None:visit(g,origin,m)
            return
        if k=='Box':size=value(n,'size','getSFVec3f',(2,2,2))
        elif k=='Sphere':
            r=value(n,'radius','getSFFloat',1.);size=(2*r,)*3
        elif k in ('Cylinder','Cone','Capsule'):
            r=value(n,'radius','getSFFloat',None)
            if r is None:r=value(n,'bottomRadius','getSFFloat',1.)
            h=value(n,'height','getSFFloat',2.)
            # A cube safely encloses primitives irrespective of axis convention.
            diameter=max(2*r,h+2*r if k=='Capsule' else h);size=(diameter,)*3
        else:raise ValueError('unsupported boundingObject type '+k)
        for p in product(*[(-v/2,v/2) for v in size]):points.append(add(origin,mv(m,p)))
    visit(node,(0.,0.,0.),IDENTITY)
    if not points:raise ValueError('empty boundingObject')
    return points

def obstacle_from_solid(node,corners,name):
    pos=node.getPosition();r=node.getOrientation();points=[add(pos,mv(r,p)) for p in corners]
    if not all(math.isfinite(v) for p in points for v in p):raise ValueError('invalid world geometry')
    low=[min(p[i] for p in points) for i in range(3)];high=[max(p[i] for p in points) for i in range(3)]
    return {'name':name,'x':(low[0]+high[0])/2,'y':(low[1]+high[1])/2,
            'sx':high[0]-low[0],'sy':high[1]-low[1],'bottom':low[2],'top':high[2]}
