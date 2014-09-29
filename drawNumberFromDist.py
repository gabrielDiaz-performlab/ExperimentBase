

def drawNumberFromDist(distType,distParams):		
	if( distType == 'fixed' ):
		return float(distParams)
	elif( distType == 'uniform'):
		return drawFromUniform(distParams)
	elif( distType == 'gaussianTruncated'):
		return drawFromGaussianTruncated(distParams)
	
	
def drawFromUniform(minMax):
	from numpy import random

	if(len(minMax)!=2):
		print('Invalid parameters for uniform distribution')
		return
	else:
		return float(minMax[0]) + random.rand()*(float(minMax[1])-float(minMax[0])) 

		
def drawFromGaussianTruncated(muSigmaClip):
	
	from numpy import random
	
	muSigmaClip = muSigmaClip
	
	mu = float(muSigmaClip[0])
	sigma = float(muSigmaClip[1])
	clip = float(muSigmaClip[2])
	
	randNum = mu + random.standard_normal()*sigma
	
	clipLowerThresh = mu - (sigma*clip)
	clipUpperThresh = mu + (sigma*clip)
	
	while( randNum < clipLowerThresh or randNum > clipUpperThresh ):
	
		randNum = mu + random.standard_normal()*sigma
	
	return randNum
	
if __name__ == "__main__":
	
	a = drawNumberFromDist('gaussianTruncated',[10,1,2])
	print str(a)