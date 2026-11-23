"""Reference state-augmented grid hybrid planner for aerial/ground travel.

Search state (x_cell, y_cell, heading_bin, motion_mode). This is a tested,
inspectable educational implementation; it is NOT a reproduction of a named
research paper's continuous-curvature State-Augmented Hybrid A* algorithm.
"""
from dataclasses import dataclass
import heapq
import itertools
import math

DIRS = ((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1))

@dataclass(frozen=True)
class PoseState:
    x: float
    y: float
    yaw_rad: float
    mode: str

class HybridStatePlanner:
    def __init__(self, grid, walk_cost=1.35, flight_cost=1.0,
                 turn_cost=0.22, transition_cost=4.0, transition_zones=()):
        self.grid = grid
        self.walk_cost = float(walk_cost)
        self.flight_cost = float(flight_cost)
        self.turn_cost = float(turn_cost)
        self.transition_cost = float(transition_cost)
        self.transition_cells = {grid.xy_to_cell(*p) for p in transition_zones}

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
                    result.append(PoseState(x,y,h*math.pi/4,mode))
                return result
            for nxt,weight in self._neighbors(cur):
                new=cost+weight
                if new+1e-9<costs.get(nxt,float('inf')):
                    costs[nxt]=new
                    predecessor[nxt]=cur
                    heapq.heappush(queue,(new+heuristic(nxt),new,next(counter),nxt))
        raise ValueError('No state-feasible path between start and goal')


def choose_safe_walk_goal(grid,position,offsets=((5,0),(4,4),(0,6),(-5,0),(0,-5))):
    """Find a short walk target in the currently available known map."""
    for dx,dy in offsets:
        target=(position[0]+dx,position[1]+dy)
        if grid.is_free(grid.xy_to_cell(*target)):
            try:
                route=HybridStatePlanner(grid).plan(position,target,'WALK','WALK')
                if len(route)>2:return target,route
            except ValueError:pass
    raise ValueError('No safe walking destination near current drone position')
