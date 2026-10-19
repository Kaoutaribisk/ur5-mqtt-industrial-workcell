import math

def frame_cell(sim):
    eye=[2.3,-3.2,2.5]
    center=[0,-.15,.65]
    def unit(v):
        size=math.sqrt(sum(x*x for x in v))
        return [x/size for x in v]
    def cross(a,b): return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
    z=unit([b-a for a,b in zip(eye,center)])
    x=unit(cross([0,0,1],z))
    y=cross(z,x)
    matrix=[value for row in range(3) for value in (x[row],y[row],z[row],eye[row])]
    sim.setObjectMatrix(sim.getObject('/DefaultCamera'),-1,matrix)
