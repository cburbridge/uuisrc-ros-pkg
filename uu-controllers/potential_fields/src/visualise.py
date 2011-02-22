from  numpy import meshgrid, cos, sin, arctan2, fabs, arange, sqrt, zeros
from matplotlib import *
from pylab import *

def potfield():
	target_x, target_y = 0.0, 0.0
	opt_dist = 2
	dist_thr = 0.5
	spacing = 0.2

	X,Y = meshgrid(arange(-4,4,spacing), arange(-4,4,spacing) )

	#tangential
	Zu_circle = zeros(X.shape)
	Zv_circle = zeros(X.shape)
	k_tan = 5.0
	for i in xrange(X.shape[0]):
		for j in xrange(X.shape[1]):
			d = sqrt( (X[i,j] - target_x)**2 + (Y[i,j] - target_y)**2 )
			if fabs(d - opt_dist ) <dist_thr:
				#slope = arctan( -(X[i,j] - target_x) / (Y[i,j] - target_y) )
				slope =  - arctan2( (X[i,j] - target_x ), (Y[i,j] - target_y) )
				Zu_circle[i,j] = -cos(slope)
				Zv_circle[i,j] = -sin(slope)

	#attractive		
	Zu_attr = zeros(X.shape)
	Zv_attr = zeros(X.shape)
	k_att = 0.5
	for i in xrange(X.shape[0]):
		for j in xrange(X.shape[1]):
			d = sqrt( (X[i,j] - target_x)**2 + (Y[i,j] - target_y)**2 )
			if d > opt_dist:
				Zu_attr[i,j] = -(X[i,j] - target_x)
				Zv_attr[i,j] = -(Y[i,j] - target_y)

	#repulsive
	Zu_rep = zeros(X.shape)
	Zv_rep = zeros(X.shape)
	k_rep = 0.5
	for i in xrange(X.shape[0]):
		for j in xrange(X.shape[1]):
			d = sqrt( (X[i,j] - target_x)**2 + (Y[i,j] - target_y)**2 )
			if d < opt_dist:
				Zu_rep[i,j] = (X[i,j] - target_x)
				Zv_rep[i,j] = (Y[i,j] - target_y)
                
	#for i in xrange(X.shape[0]):
		#for j in xrange(X.shape[1]):
			#d = sqrt( (X[i,j] - target_x)**2 + (Y[i,j] - target_y)**2 )
			#d0 = opt_dist
			##k_rep = 10.
			#if d < d0 and d != 0:
				##Zu_rep[i,j] = k_rep*(1./d - 1/d0) * 1./(d**2) * (X[i,j] - target_x)/d
				##Zv_rep[i,j] = k_rep*(1./d - 1/d0) * 1./(d**2) * (Y[i,j] - target_y)/d
				#Zu_rep[i,j] = (X[i,j] - target_x)
                #Zv_rep[i,j] = (Y[i,j] - target_y)
		
			
	Zu = k_tan * Zu_circle + k_att * Zu_attr + k_rep * Zu_rep
	Zv = k_tan * Zv_circle + k_att * Zv_attr + k_rep * Zv_rep
	#Zu = Zu_circle
	#Zv = Zv_circle
	
	#Zu[Zu>1.0] = 1.0
	#Zu[Zu<-1.0] = -1.0
	#Zv[Zv>1.0] = 1.0
	#Zv[Zv<-1.0] = -1.0
	
	Zu/=sqrt( Zu**2 + Zv**2 )
	Zv/=sqrt( Zu**2 + Zv**2 )
	
	##Zu*=0.1
	##Zv*=0.1

	return X,Y,Zu,Zv

X,Y,Zu,Zv = potfield()
figure()
quiver(X,Y,Zu,Zv,units='width')
show()