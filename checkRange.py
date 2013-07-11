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
            overlapRangeIndex = overlapBndPts[i][0]
            overlapRangeSide = overlapBndPts[i][1]
            prevOverlapRangeIndex = overlapBndPts[i-1][0]
            prevOverlapRangeSide = overlapBndPts[i-1][1]
            
            bndPt = overlapBndPts[i][2] #rangeBndPts(overlapRangeIndex+overlapRangeSide);
            bndDist = overlapBndPts[i][4] #rangeBndPtsDist(overlapRangeIndex,1+overlapRangeSide);
            prevBndPt = overlapBndPts[i-1][2]
            prevBndDist = overlapBndPts[i-1][3] #rangeBndPtsDist(prevOverlapRangeIndex,1+prevOverlapRangeSide);
            
            overlapRangeLeft = rangeBndPts[overlapRangeIndex+left]; #this is where you need to check whether a -1 is necessary or not
            overlapRangeLeftDist = rangeBndPtsDist[overlapRangeIndex,1+left];
            overlapRangeRight = rangeBndPts[overlapRangeIndex+right];
            overlapRangeRightDist = rangeBndPtsDist[overlapRangeIndex,1+right];
			
	    if i==numOverlapBndPts-1 && overlapRangeSide==right 
	      && overlapRangeRight == furthestLeftRightBnd:
				#warning('break here');
            
            overlapRangeRightmostApplicableBnd = prevBndPt;
            
            bndPtOnEdge = bndPt + dirSign * gripperWidthGamma; #multiplying matrices?
            #if the bndPtOnEdge is outside of the actual edge range,
            #truncate it
            if (bndPtOnEdge - leftBound)*dirSign < 0:
                bndPtOnEdge = leftBound
            elif (rightBound - bndPtOnEdge)*dirSign < 0:
                bndPtOnEdge = rightBound
            bndPtEdgeDist = np.maximum(leftDist,leftDist +
                ((bndPtOnEdge - leftBound)/rangeWidth) * (rightDist-leftDist)) #assuming these are vectors.
            
            #if the range is always fully covered by the gripper, skip it,
            #and all closer ranges.
            if (furthestRightOverlapGamma-overlapRangeLeft)*dirSign <= 0:
                break;
            
            #if the distance at the left side of the gripper is less than on
            #the right, it contacts the edge, so success over the whole range
            #closure on the edge
            if (overlapRangeSide == left) && (bndDist < bndPtEdgeDist):
                #range success is closure on edge
                if flip:
                    #checkedRanges(end+1) = checkConvexVertex([overlapRangeLeft,overlapRangeRight],rangeIndex,left);
                    np.append(checkedRanges,checkConvexVertex(np.concatenate(overlapRangeLeft,overlapRangeRightmostApplicableBnd),leftDist,rangeIndex,left));
                else:
                    
                    sortedBnds = np.sort(np.concatenate((overlapRangeLeft,overlapRangeRightmostApplicableBnd),axis=1) + dirSign*gripperWidthGamma)
                    sortInd = np.argsort(np.concatenate((np.concatenate((overlapRangeLeft,overlapRangeRightmostApplicableBnd),axis=1) + dirSign*gripperWidthGamma)
                    
                    dists = leftDist + (rightDist-leftDist) * (sortedBnds-leftBound) / (rightBound-leftBound)
                    
                    checkedRanges = array([checkedRanges,checkEdgeContact(sortedBnds,dists,frictionSuccess,edgeIndex,DONT_USE_OFFSET]))
                continue;
            
            if overlapRangeSide == left:
                #rightBnd = overlapRangeRight;
                #rightBndDist = overlapRangeRightDist;
                
                #check if left bound is too far, truncate
                if (overlapRangeLeft-furthestLeftGamma)*dirSign < 0:
                    overlapRangeLeftDist = overlapRangeRightDist +
                        (overlapRangeLeftDist-overlapRangeRightDist) *
                        (furthestLeftGamma-overlapRangeRight) / (overlapRangeLeft-overlapRangeRight)
                    overlapRangeLeft = furthestLeftGamma
                
                #check if right bound is too close, truncate
                if (overlapRangeRight-furthestRightOverlapGamma)*dirSign > 0:
                    overlapRangeRightDist = overlapRangeRightDist +
                        (overlapRangeLeftDist-overlapRangeRightDist) *
                        (furthestRightOverlapGamma-overlapRangeRight) / (overlapRangeLeft-overlapRangeRight)
                    overlapRangeRight = furthestRightOverlapGamma
                    overlapRangeRightmostApplicableBnd = overlapRangeRight
                elif (overlapRangeRightmostApplicableBnd-furthestRightOverlapGamma)*dirSign > 0:
                    overlapRangeRightmostApplicableBnd = furthestRightOverlapGamma;
                
                #if the previous boundary is higher than the right side of this
                #range, find where this range gets higher than that.
                if prevBndDist > overlapRangeRightDist:
                    splitPt = overlapRangeLeft + 
                        (prevBndDist-overlapRangeLeftDist)/(overlapRangeRightDist - overlapRangeLeftDist)
                        *(overlapRangeRight-overlapRangeLeft);
                    
                    #analyze right of split
                    checkedRanges = np.array([checkedRanges,checkGapRange(splitPt,overlapRangeRightmostApplicableBnd,prevBndDist,prevOverlapRangeIndex,prevOverlapRangeSide)])
                    
                    #rightBnd = splitPt
                    #rightBndDist = prevBndDist
                    overlapRangeRight = splitPt
                    overlapRangeRightDist = prevBndDist
                    overlapRangeRightmostApplicableBnd = splitPt
                
                if flip:
                    checkedRanges = np.array([checkedRanges checkGapRangeRight(...
                        overlapRangeLeft,overlapRangeLeftDist,overlapRangeRight,overlapRangeRightDist,overlapRangeRightmostApplicableBnd,overlapRangeIndex)])
                        #overlapRangeLeft,overlapRangeLeftDist,rightBnd,rightBndDist,overlapRangeIndex)])
                
                else:
                    rightEdgePt = overlapRangeRight+ dirSign * gripperWidthGamma;
                    rightEdgeDist = np.max(leftDist,leftDist +
                        (np.abs(rightEdgePt - leftBound)/rangeWidth) * (np.abs(rightDist-leftDist))) #Check that np has max and abs?
                    
                    if rightDist < rightEdgeDist and abs(rightDist - rightEdgeDist) > 1e-10:
                        #find intersection where distance to the edge matches
                        #distance to the overlapped range
                        leftEdgePt = overlapRangeLeft + dirSign * gripperWidthGamma
                        leftEdgeDist = leftDist + 
                            (np.abs(leftEdgePt - leftBound)/rangeWidth) * (np.abs(rightDist-leftDist))
                        
                        dx = overlapRangeRight - overlapRangeLeft;
                        line1 = [overlapRangeLeft, overlapRangeLeftDist, dx, overlapRangeRightDist-overlapRangeLeftDist];
                        line2 = [overlapRangeLeft, rightEdgeDist, dx, rightEdgeDist-leftEdgeDist];
                        
                        pt = intersectLines(line1,line2);
                        splitPt = pt(1);
                        splitDist = pt(2);
                        
                        if (splitPt-overlapRangeLeft)*dirSign < 0
                            splitPt = overlapRangeLeft;
                            splitDist = overlapRangeLeftDist;
                        end
                        
                        sortedBnds = sort([splitPt,overlapRangeRightmostApplicableBnd]+dirSign*gripperWidthGamma);
                        dists = leftDist + (rightDist-leftDist) * (sortedBnds-leftBound) / (rightBound-leftBound);
                        
                        checkedRanges = [checkedRanges checkEdgeContact(sortedBnds,dists,frictionSuccess,edgeIndex,DONT_USE_OFFSET)];
                        
                        %rightBnd = splitPt;
                        %rightBndDist = splitDist;
                        overlapRangeRight = splitPt;
                        overlapRangeRightDist = splitDist;
                        overlapRangeRightmostApplicableBnd = splitPt;
                    end
                    
                    if overlapRangeLeft ~= overlapRangeRightmostApplicableBnd
                        %success is friction on what's left
                        overlapRangeFrictionSuccess = rangeFrictionSuccess(overlapRangeIndex);
                        [sortedBnds,sortInd] = sort([overlapRangeLeft,overlapRangeRightmostApplicableBnd]+dirSign*gripperWidthGamma);
                        distBnds = [overlapRangeLeft, dirSign * min(dirSign*[overlapRangeRight,overlapRangeRightmostApplicableBnd])];
                        dists = overlapRangeLeftDist + (overlapRangeRightDist-overlapRangeLeftDist) * (distBnds-overlapRangeLeft) / (overlapRangeRight-overlapRangeLeft);
                        
                        checkedRanges = [checkedRanges checkEdgeContact(sortedBnds,dists(sortInd),overlapRangeFrictionSuccess,rangeEdgeIndex(overlapRangeIndex),USE_OFFSET)];
                    end
                end
            else %overlapRangeSide == right
                %if it's the last bndPt, the rest has already been taken
                %care of
                %{
                if overlapInd==size(overlapBndPts,1)
                    continue;
                    
                    if bndPt == furthestLeftGamma
                        continue;
                    end
                    
                    furthestLeftInThisRange = dirSign*max(dirSign*[furthestLeftGamma,overlapRangeLeft]);
                    furthestRightInThisRange = dirSign*min(dirSign*[bndPt,furthestRightOverlapGamma]);
                    
                    checkedRanges = [checkedRanges checkGapRange(furthestLeftInThisRange,furthestRightInThisRange,bndDist,overlapRangeIndex,overlapRangeSide)];
                    continue;
                end
                %}
                
                %if this is on the right end of a range, and the previous point
                %is the left edge of the adjacent range, it must therefore be
                %in the same spot.  So we skip it, and it will get picked up by
                %the next range
                if prevOverlapRangeIndex == overlapRangeIndex + dirSign &&...
                        prevOverlapRangeSide == left
                    continue;
                end
                
                if (overlapRangeRightmostApplicableBnd-furthestRightOverlapGamma)*dirSign > 0
                    overlapRangeRightmostApplicableBnd = furthestRightOverlapGamma;
                end
                
                %check based on prev point
                %checkedRanges = [checkedRanges checkGapRange(bndPt,prevBndPt,prevBndDist,prevOverlapRangeIndex,prevOverlapRangeSide)];
                checkedRanges = [checkedRanges checkGapRange(bndPt,overlapRangeRightmostApplicableBnd,prevBndDist,prevOverlapRangeIndex,prevOverlapRangeSide)];
            end
        end

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