import numpy, math

USE_OFFSET = True
DONT_USE_OFFSET = False

def checkRange(self,segments,unitWidth,gripperWidthGamma,gripperOpening,useConvexVertex,useSlip,supportRange):
    checkedRanges = CheckedRanges()
    overlapBndPts = np.zeros([0,4]) #check for right orientation
    
    if ((self.name == 'unknown_shape_sample798') &&  segments[0].edge == 2):
        #warning('break here')
    segments.append(Segment(segments[len(segments)-1)].bnd[2] + array([0,gripperWidthGamma]),array([0,0]),array([NaN]),array([0]))
    
    rangeBndPts = unique([elem.bnd for elem in segments])
    rangeBndPts = np.zeros(len(segments),2)
    i = 0
    while i < len(segments):
        rangeBndPtsDist[i] = segments[i].dist
        i += 1 
    rangeEdgeIndex = [elem.edge for elem in segments]
    
    phiMax = arctan(this.mu)
    
    rangeFrictionSuccess = [abs(elem.angle) for elem in segments]
    #is angle an array or a number?
    
    i = 0
    while i < len(segments):
        edgeIndex = rangeEdgeIndex[i]
        
        frictionSuccess = rangeFrictionSuccess[i]
        
        rangeWidth = abs_array(rangeBndPts(i+1) - rangeBndPts(i))
        #how is i+1 going to stay within range?
        
        if rangeBndPtsDist[i][0] > rangeBndPtsDist[i][1]:
            flip = True
            dirSign = 1
            left = 0
            right = 1
        else:
            flip = False
            dirSign = 1
            left = 0
            right = 1
        leftBound = rangeBndPts[i+left]
        rightBound = rangeBndPts[i+right]
        leftDist = rangeBndPtsDist[i][left+1]
        rightDist = rangeBndPtsDist[i][right+1]
        
        #first, check if there's an area of the edge where the gripper doesn't
        #overlap anything
        if rangeWidth > gripperWidthGamma+1e-6:
            overlapRightBound = leftBound + dirSign * gripperWidthGamma #incorrect - need different add operation
            overlapRightBoundDist = leftDist + (rightDist - leftDist) * (overlapRightBound - leftBound)/ (rightBound-leftBound)
            #on-edge process
            
            np.concatenate((sortedBnds,sortInd),axis=1)
            
    
def unique(array_list):
    if len(array_list) <= 1:
        return array_list
    i = 0;
    new_list = array_list
    while i < len(array_list)-1:
        if(array_equal(array_list[i],array_list[i+1])):
            new_list.remove(array_list[i])
        i += 1
    return new_list

def abs_array(array):
    if len(array.shape) == 1:
        i = 0
        while i < len(array):
            array[i] = abs(array[i])
            i += 1
    elif len(array.shape) == 2:
        i = 0
        while i < len(array):
            j = 0
            while j < len(array[i]):
                array[i][j] = abs(array[i][j])
                j += 1
            i += 1

class CheckedRanges:
    def __init__():
        self.bnd = False
        self.dist = False
        self.valid = False
        self.ind = False
        self.type = False
        self.reason = False

class Segment:
    def __init__(bnd = False, dist = False, edge = False, angle = False):
        self.bnd = bnd
        self.dist = dist
        self.edge = edge
        self.angle = angle