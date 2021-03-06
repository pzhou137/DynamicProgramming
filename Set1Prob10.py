import numpy as np
import collections

class DPsys(object):

    def __init__(self):

        self.states = range(11)
        self.timesteps = range(10)

        self.controls = collections.defaultdict(list)
        for state in self.states:
            self.controls[state] = list(range( 0 - state, 11 - state))

        self.num_timesteps = len(self.timesteps) # k = 0, 1, ..., 9
        self.num_states = len(self.states)

        self.p_wk = 1./3 # prob for wk taking value 1

        self.J = np.zeros( (self.num_states, self.num_timesteps + 1) )
        self.best_controls = np.zeros( (self.num_states, self.num_timesteps), dtype=np.int8 )  

    def forward_evolution (self, x, u, w):
        return x + u * w
    
    def local_cost(self, x, u, k): 
        xref = (k - 5) ** 2
        return (x - xref) ** 2 + u ** 2 # this is w-independent

    def DPA(self):
        """
        :return: 
            list[int], optimal cost for all initial states
            list[list[int]], optimal policy for each initial state and each timestep
        """
        for i, state in enumerate(self.states):
            self.J[i][-1] = state ** 2

        for k in reversed(self.timesteps):
            for i, x in enumerate(self.states):
                localmap = collections.defaultdict(int) # a map from cost to control
                for u in self.controls[x]:
                    cost = ( self.local_cost(x, u, k) 
                        + self.p_wk * self.J[self.forward_evolution(x, u, 1)][k+1] 
                        + (1-self.p_wk) * self.J[self.forward_evolution(x, u, 0)][k+1])
                    localmap[cost] = u
                self.J[i][k] = min(localmap.keys())
                self.best_controls[i][k] = localmap.get( self.J[i][k] )
        return self.J, self.best_controls

    def getPath(self, i, w = 1):
        """
        :input i: int, state index at timestep 0
        :return: list[int], the path under optimal policy
        """
        x_curr = self.states[i]
        path = [x_curr]
        for k in self.timesteps:
            u = self.best_controls[x_curr][k]
            x_next = self.forward_evolution(x_curr, u, w)
            path.append(x_next)
            x_curr = x_next
        return path



def main():
    mysys = DPsys()
    
    optimal_cost, optimal_policy = mysys.DPA()
    
    print(optimal_cost[:,0])
    print(optimal_policy)
    for i in range(mysys.num_states):
        print(mysys.getPath(i))

if __name__ == "__main__":
    main()

        



