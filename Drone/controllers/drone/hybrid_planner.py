"""Land-air Hybrid A* with fixed mode heights and a planar grid fallback."""
from dataclasses import dataclass
import heapq
import itertools
import math
from hybrid_astar import Pose, arc_poses

DIRS = ((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1))

@dataclass(frozen=True)
class PoseState:
    x: float
    y: float
    yaw_rad: float
    mode: str
    z: float = 0.0

class HybridStatePlanner:
    def __init__(self, grid, walk_cost=1.35, flight_cost=1.0,
                 turn_cost=0.22, transition_cost=4.0, transition_zones=(),
                 ground_height=0.6, flight_height=13.0):
        self.grid = grid
        self.walk_cost = float(walk_cost)
        self.flight_cost = float(flight_cost)
        self.turn_cost = float(turn_cost)
        self.transition_cost = float(transition_cost)
        self.transition_cells = {grid.xy_to_cell(*p) for p in transition_zones}
        self.ground_height = ground_height
        self.flight_height = flight_height

    def _land_air_plan(self, start, goal, start_mode, goal_mode, yaw):
        grid = self.grid
        if not self.transition_cells:
            raise ValueError('No authorized transition site')
        initial = PoseState(*start, yaw, start_mode,
                            self.flight_height if start_mode == 'FLIGHT' else self.ground_height)
        step = grid.resolution

        def key(p):
            return (*grid.xy_to_cell(p.x,p.y), round(p.yaw_rad*12/math.pi)%24, p.z, p.mode)

        def estimate(p):
            distance = max(0.0, math.hypot(p.x-goal[0],p.y-goal[1])-step)
            return distance*min(self.walk_cost,self.flight_cost) + (
                self.transition_cost if p.mode != goal_mode else 0.0)

        def successors(p):
            origin = (p.x,p.y)
            cell = grid.xy_to_cell(*origin)
            if self.is_transition_cell(cell):
                mode = 'WALK' if p.mode == 'FLIGHT' else 'FLIGHT'
                height = self.ground_height if mode == 'WALK' else self.flight_height
                yield PoseState(*origin,p.yaw_rad,mode,height), self.transition_cost+abs(height-p.z)
            if p.mode == 'FLIGHT':
                for dx,dy in DIRS:
                    x,y = grid.cell_to_xy((cell[0]+dx,cell[1]+dy))
                    if grid.clear_segment(origin,(x,y)):
                        yield PoseState(x,y,math.atan2(y-p.y,x-p.x),p.mode,p.z), math.hypot(x-p.x,y-p.y)*self.flight_cost
            else:
                for length in (step,-step):
                    for curvature in (-0.7,0.0,0.7):
                        arc = arc_poses(Pose(*origin,p.yaw_rad),curvature,length)
                        if all(grid.is_free(grid.xy_to_cell(q.x,q.y)) for q in arc):
                            q = arc[-1]
                            yield PoseState(q.x,q.y,q.yaw,p.mode,p.z), step*self.walk_cost+self.turn_cost*abs(curvature*length)

        nodes = [(initial,None)]
        best = {key(initial):0.0}
        queue = [(estimate(initial),0.0,0)]
        while queue:
            _,cost,index = heapq.heappop(queue)
            p,_ = nodes[index]
            if cost != best[key(p)]:
                continue
            if p.mode == goal_mode and math.hypot(p.x-goal[0],p.y-goal[1]) <= step:
                route = []
                while index is not None:
                    p,index = nodes[index]
                    route.append(p)
                return route[::-1]
            for child,weight in successors(p):
                k = key(child)
                total = cost + weight
                if total < best.get(k,float('inf')):
                    best[k] = total
                    heapq.heappush(queue,(total+estimate(child),total,len(nodes)))
                    nodes.append((child,index))
        raise ValueError('No state-feasible land-air route')

    def is_transition_cell(self, cell):
        # Only explicitly authorized safe sites permit mode changes.
        return cell in self.transition_cells and self.grid.is_free(cell)

    @staticmethod
    def angle_index(yaw):
        return int(round(yaw / (math.pi/4))) % 8

    def _neighbors(self, state):
        ix,iy,h,mode=state
        if mode=='WALK':
            # Walking can turn in place and take heading-constrained steps.
            for dh in (-1,1):
                yield (ix,iy,(h+dh)%8,mode), self.turn_cost + 0.1
            headings=[(h,0.0),((h-1)%8,self.turn_cost),((h+1)%8,self.turn_cost)]
            factor=self.walk_cost
        else:
            headings=[(j,self.turn_cost*min((j-h)%8,(h-j)%8)) for j in range(8)]
            factor=self.flight_cost
        for j, extra in headings:
            dx,dy=DIRS[j]
            target=(ix+dx,iy+dy)
            if not self.grid.is_free(target): continue
            if dx and dy and not (self.grid.is_free((ix+dx,iy)) and
                                  self.grid.is_free((ix,iy+dy))):
                continue
            yield (target[0],target[1],j,mode),factor*math.hypot(dx,dy)*self.grid.resolution + extra
        if self.is_transition_cell((ix,iy)):
            other='WALK' if mode=='FLIGHT' else 'FLIGHT'
            yield (ix,iy,h,other),self.transition_cost

    def plan(self,start,goal,start_mode='FLIGHT',goal_mode='FLIGHT',start_yaw=0.0):
        if start_mode not in ('FLIGHT','WALK') or goal_mode not in ('FLIGHT','WALK'):
            raise ValueError('Mode must be FLIGHT or WALK')
        s=self.grid.xy_to_cell(*start)
        g=self.grid.xy_to_cell(*goal)
        if not self.grid.is_free(s) or not self.grid.is_free(g):
            raise ValueError('Start or goal is obstructed')
        if start_mode != goal_mode:
            return self._land_air_plan(start,goal,start_mode,goal_mode,start_yaw)
        state=(s[0],s[1],self.angle_index(start_yaw),start_mode)
        def heuristic(q):
            remaining=math.hypot(q[0]-g[0],q[1]-g[1])*self.grid.resolution
            modes = self.transition_cost if q[3]!=goal_mode else 0.0
            return remaining*min(self.walk_cost,self.flight_cost)+modes
        counter=itertools.count()
        queue=[(heuristic(state),0.0,next(counter),state)]
        costs={state:0.0}; predecessor={}
        while queue:
            _,cost,_,cur=heapq.heappop(queue)
            if cost > costs.get(cur,float('inf'))+1e-9:continue
            if cur[:2]==g and cur[3]==goal_mode:
                out=[cur]
                while out[-1]!=state:out.append(predecessor[out[-1]])
                out.reverse()
                result=[]
                for ix,iy,h,mode in out:
                    x,y=self.grid.cell_to_xy((ix,iy))
                    height = self.ground_height if mode == 'WALK' else self.flight_height
                    result.append(PoseState(x,y,h*math.pi/4,mode,height))
                return result
            for nxt,weight in self._neighbors(cur):
                new=cost+weight
                if new+1e-9<costs.get(nxt,float('inf')):
                    costs[nxt]=new
                    predecessor[nxt]=cur
                    heapq.heappush(queue,(new+heuristic(nxt),new,next(counter),nxt))
        raise ValueError('No state-feasible path between start and goal')


def choose_safe_walk_goal(grid,position,offsets=((5,0),(4,4),(0,6),(-5,0),(0,-5)),
                          *, route_for_goal=None):
    """Find a short ground target reachable by the requested planner."""
    if route_for_goal is None:
        planner = HybridStatePlanner(grid)
        route_for_goal = lambda target: planner.plan(position,target,'WALK','WALK')
    for dx,dy in offsets:
        target=(position[0]+dx,position[1]+dy)
        if grid.is_free(grid.xy_to_cell(*target)):
            try:
                route=route_for_goal(target)
                if len(route)>2:return target,route
            except ValueError:pass
    raise ValueError('No safe ground destination near current drone position')
