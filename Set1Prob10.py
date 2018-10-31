import numpy as np
import collections

class DPsys(object):
    states = range(11)
    timesteps = range(10)
    controls = collections.defaultdict(list)
    for state in states:
        controls[state] = list(range( 0 - state, 11 - state))

    num_timesteps = len(timesteps) # k = 0, 1, ..., 9
    num_states = len(set_states)
    num_controls = 11
    p_wk = 1./3 # prob for wk taking 1

    def forward_evolution (self, x, u, w):
        return x + u * w
    def backward_evolution (self, x, u, w):
        return x - u * w

    def xref(self, k):
        return (k-5) ** 2
    
    def cost_to_go(self, x, u, w, k):
        return (x - self.xref(k)) ** 2 + u ** 2

    def DPA(self):

        """
        :return: 
            list[int], optimal cost for all initial states
            list[list[int]], optimal policy for each initial state and each timestep
        """

        J = [ [0] * (self.num_timesteps + 1) ] * num_states
        for state in self.states:
            J[state][-1] = state ** 2

        for k in reversed(self.timesteps):

        



