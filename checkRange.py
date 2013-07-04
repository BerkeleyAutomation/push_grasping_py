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
            sortedBnds = np.sort(np.concatenate((overlapRightBound,rightBound),axis=1))
            sortInd = np.argsort(np.concatenate((overlapRightBound,rightBound),axis=1))
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
                #make sure you should be doing "append" for end + 1
                np.append(overlapBndPts,np.concatenate((overlapRangeInd,left,furthestLeftInRange,furthestLeftDistInRange), axis =1)) #CHECK AXIS!
            else:
                if rangeRightDist >= maxDist and rangeRightDist > leftDist
                    maxDist = rangeRightDist;
                    overlapBndPts[end+1] = [overlapRangeInd,right,rangeRight,rangeRightDist]; #Check array set
        
        if (rangeLeft-furthestLeftGamma)*dirSign <= 0:
            break;
            
            i += 1
    
    if size(overlapBndPts) >= 2
		#if last side is right and second-to-last is left and they are adjacent,
		#and the distances are the same, take off the last one.
		end = len(overlapBndPts)
		if np.diff(array([a[0] for a in overlapBandPts[end-2:end]])) == -1 and
				overlapBndPts[end-1][1]==right and overlapBndPts[end-2][1]==left
				and np.diff(array([a[3] for a in overlapBndPts[end-2:end]))) == 0: #check that last one...
			overlapBndPts[end-1] = [];

    clear rangeLeft
    clear rangeRight
    clear rangeLeftDist
    clear rangeRightDist;
    
    numOverlapBndPts = overlapBndPts.shape;
    
    if numOverlapBndPts == 0:
        if flip:
            np.append(checkedRanges,checkConvexVertex(np.concatenate(leftBound,overlapRightBound), leftDist,rangeIndex,left));
        else:
            sortedBnds = np.sort(np.concatenate((leftBound,overlapRightBound),axis=1))
            sortInd = np.argsort(np.concatenate((leftBound,overlapRightBound),axis=1))
            dists = np.concatenate(leftDist,overlapRightBoundDist);
            
            checkedRanges = array([checkedRanges,checkEdgeContact(sortedBnds,dists[sortInd],frictionSuccess,edgeIndex,DONT_USE_OFFSET)]);
        #TODO: slip-stick (check at phiMax for obstructions)
    else:
        #if the gripper sticks out beyond all the ranges
        furthestLeftOverlapRangeIndex = overlapBndPts[overlapBndPts.shape-1][0];
        furthestLeftOverlapRangeSide = overlapBndPts[overlapBndPts.shape-1][1];
        furthestLeftOverlapBnd = overlapBndPts[overlapBndPts.shape-1][2];
        furthestLeftOverlapBndDist = overlapBndPts[overlapBndPts.shape-1][3];
        if (furthestLeftGamma-furthestLeftOverlapBnd)*dirSign < 0:
            #the right boundary is either the left edge of the first range,
            #or the furthest right that the left edge of the gripper ever
            #gets
			#leftEdgeOfFirstRange = rangeBndPts(furthestLeftOverlapRangeIndex+left);
            #furthestLeftRightBnd = dirSign*min(dirSign*[leftEdgeOfFirstRange,furthestRightOverlapGamma]);
            furthestLeftRightBnd = dirSign*min(dirSign*np.concatenate(furthestLeftOverlapBnd,furthestRightOverlapGamma)); #check matrix multiplication
			#TODO: handles vertex closure right?
            checkedRanges = np.array([checkedRanges,checkGapRange(furthestLeftGamma,furthestLeftRightBnd,furthestLeftOverlapBndDist,furthestLeftOverlapRangeIndex,furthestLeftOverlapRangeSide)]);
		else:
		    furthestLeftRightBnd = furthestLeftOverlapBnd;
        
        #if all the ranges have been covered, don't bother going through
        #them
        if (furthestRightOverlapGamma-furthestLeftOverlapBnd)*dirSign < 0:
            continue;
        
         #for overlapInd=numOverlapBndPts:-1:2
         #Check when you need to end loop (put at 1 for now)
         i = numOverlapBndPts-1
         while i > 1:  
            overlapRangeIndex = overlapBndPts[i][0];
            overlapRangeSide = overlapBndPts(overlapInd,2);
            prevOverlapRangeIndex = overlapBndPts(overlapInd-1,1);
            prevOverlapRangeSide = overlapBndPts(overlapInd-1,2);
            
            bndPt = overlapBndPts(overlapInd,3);%rangeBndPts(overlapRangeIndex+overlapRangeSide);
            bndDist = overlapBndPts(overlapInd,4);%rangeBndPtsDist(overlapRangeIndex,1+overlapRangeSide);
            prevBndPt = overlapBndPts(overlapInd-1,3);
            prevBndDist = overlapBndPts(overlapInd-1,4);%rangeBndPtsDist(prevOverlapRangeIndex,1+prevOverlapRangeSide);
            
            overlapRangeLeft = rangeBndPts(overlapRangeIndex+left);
            overlapRangeLeftDist = rangeBndPtsDist(overlapRangeIndex,1+left);
            overlapRangeRight = rangeBndPts(overlapRangeIndex+right);
            overlapRangeRightDist = rangeBndPtsDist(overlapRangeIndex,1+right);
			
			if overlapInd==numOverlapBndPts...
					&& overlapRangeSide==right...
					&& overlapRangeRight == furthestLeftRightBnd
				%warning('break here');
			end
            
            overlapRangeRightmostApplicableBnd = prevBndPt;
            
            bndPtOnEdge = bndPt + dirSign * gripperWidthGamma;
            %if the bndPtOnEdge is outside of the actual edge range,
            %truncate it
            if (bndPtOnEdge - leftBound)*dirSign < 0
                bndPtOnEdge = leftBound;
            elseif (rightBound - bndPtOnEdge)*dirSign < 0
                bndPtOnEdge = rightBound;
            end
            bndPtEdgeDist = max(leftDist,leftDist + ...
                ((bndPtOnEdge - leftBound)/rangeWidth) * (rightDist-leftDist));
            i--;


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