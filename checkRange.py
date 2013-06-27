import numpy, math

USE_OFFSET = True
DONT_USE_OFFSET = False

def checkRange(self,segments,unitWidth,gripperWidthGamma,gripperOpening,useConvexVertex,useSlip,supportRange):
    checkedRanges = CheckedRanges()
    overlapBndPts = np.zeros([0,4]) #check for right orientation
    
    if ((self.name == 'unknown_shape_sample798') &&  segments[0].edge == 2):
        #warning('break here')
    segments.append(Segment(segments[len(segments)-1)].bnd[2] + array([0,gripperWidthGamma]),array([0,0]),array([NaN]),array([0]))
    
    rangeBndPts = np.unique([elem.bnd for elem in segments])
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
        
        rangeWidth = np.absolute(rangeBndPts(i+1) - rangeBndPts(i))
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
            sortedBnds = np.sort(np.concatenate((sortedBnds,sortInd),axis=1))
            sortInd = np.argsort(np.concatenate((sortedBnds,sortInd),axis=1))
            if flip:
                bndsForDist = sortedBnds - dirSign * gripperWidthGamma
            else:
                bndsForDist = sortedBnds
            dists = leftDist + (rightDist - leftDist) * (bndsForDist - leftBound) / (rightBound - leftBound)
            
            checkedRanges = array([checkedRanges,checkEdgeContact(sortedBnds,distd,frictionDuccess,edgeIndex,flip)])
            #TODO: slip-stick
        else:
            overlapRightBound = rightBound
            overlapRightBoundDist = rightDist
        
        furthestLeftGamma = leftBound - dirSign * gripperWidthGamma
        furthestRightOverlapGamma = overlapRightBound - dirSign * gripperWidthGamma
        overlapBndPts = np.zeros([0,4]) #check orientation
        
        overlapRangeToCheck = np.arange(rangeIndex-2,0,-1) #check matlab/numpy range correlation
        
        #find boundary points that the gripper overlaps when it's at the left vertex on
        #the edge that are higher than any boundary points to their right
        maxDist = -inf
        i = 0
        while i < len(overlapRangeToCheck):
            rangeLeft = rangeBndPts[overlapRangeInd+left]
            rangeRight =  rangeBndPts[overlapRangeInd+right]
            rangeLeftRightDist = rangeBndPtsDist[overlapRangeInd][left+1]
            rangeRightDist = rangeBndPtsDist[overlapRangeInd][right+1]
            
            if rangeLeftDist >= rangeRightDist:
                if rangeLeft - furthestLeftGamma < 0
                furthestLeftInRange = furthestLeftGamma;
                furthestLeftDistInRange = rangeLeftDist + (rangeRightDist-rangeLeftDist) * (furthestLeftGamma-rangeLeft) / (rangeRight-rangeLeft)
            else:
                furthestLeftInRange = rangeLeft
                furthestLeftDistInRange = rangeLeftDist
            if furthestLeftDistInRange >= maxDist && furthestLeftDistInRange > leftDist
                maxDist = furthestLeftDistInRange
                overlapBndPts[end+1] = np.concatenate((overlapRangeInd,left,furthestLeftInRange,furthestLeftDistInRange), axis =1) #CHECK AXIS!
            else:
                if rangeRightDist >= maxDist && rangeRightDist > leftDist
                    maxDist = rangeRightDist;
                    overlapBndPts(end+1,:) = [overlapRangeInd,right,rangeRight,rangeRightDist];
            end
        end
        
        if (rangeLeft-furthestLeftGamma)*dirSign <= 0
            break;
        end
	end
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