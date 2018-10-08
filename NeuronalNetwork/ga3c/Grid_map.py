import numpy as np


class GridMap:

    def __init__(self, input_laser):
        self.state = self.normalise(input_laser)
        self.height = 100 #self.state.size
        self.witdh=self.state.size
        self.States_map= self.S_map()
        self.Reward_map= self.R_map()


    def S_map_(self):
        state = np.reshape(self.state, (self.witdh))
        States_map = np.zeros((1,self.height))
        for s_ in state:
            if int(((s_*200)* self.height)/100)<self.height:
                zeros = int(((s_ * 200) * self.height) / 100)
            else:
                zeros  = int(((s_ * 100) * self.height) / 100)
            if (self.height - zeros) > 0 :
                ones = int(self.height - zeros)
            else:
                ones = 0
            col = np.zeros((zeros), dtype=float)
            col = np.append(col, np.ones((ones), dtype=float))
            col = np.reshape(col,(1,self.height))
            States_map=np.append(States_map,col,axis=0)
        States_map= np.rot90(np.reshape(np.delete(States_map, 0, 0), (self.witdh, self.height)))
        return States_map

    def S_map(self):
        state = np.reshape(self.state, (self.witdh))
        States_map = np.zeros((1,self.height))
        for s_ in state:
            zeros = int(s_ * self.height )
            ones = int(self.height - zeros)
            col = np.zeros((zeros), dtype=float)
            col = np.append(col, np.ones((ones), dtype=float))
            col = np.reshape(col,(1,self.height))
            States_map=np.append(States_map,col,axis=0)
        States_map= np.rot90(np.reshape(np.delete(States_map, 0, 0), (self.witdh, self.height)))
        return States_map

    def R_map(self):
        Reward_map=np.zeros((1,self.height))
        States_map= np.rot90(self.States_map,1)

        #States_map=np.transpose(self.States_map)
        for s in States_map:
            ones= np.count_nonzero(s)-1
            zeros = 0
            if (ones) <0 :
                ones = 0
                zeros-=1

            zeros += np.count_nonzero(s-1)
            collisions = np.ones((ones),dtype=float) * (-1)
            free = np.ones((zeros),dtype=float) * (-0.04)

            if zeros==self.height:
                ziel=zeros*1
            elif zeros< self.height and zeros >= ((self.height)/2):
                ziel = zeros * 0.5
            else:
                ziel= ones * (-1)
            R_col = np.array(np.append(np.append(collisions,ziel ),free))
            R_col = np.reshape(R_col,(1,self.height))
            Reward_map=np.append(Reward_map,R_col,axis=0)
        Reward_map = np.delete(Reward_map,0,0)
        Reward_map = np.rot90(Reward_map,-1)

        return Reward_map
    def normalise(self,s):
        s = np.array ([s])
        shape = s.shape
        max_min = np.amax(s) - np.amin(s)
        min = np.amin(s)
        if not max_min==0:
            s = (s - min) / max_min
            #s = (s + 1) / 2
        else:
            s = (s - min)
            #s = (s + 1) / 2
        return np.reshape(s, shape)
