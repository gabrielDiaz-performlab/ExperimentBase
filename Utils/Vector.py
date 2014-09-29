#!/usr/bin/env python
# 
# a python Vector class
# A. Pletzer 5 Jan 00/11 April 2002
#
import math

"""
A list based Vector class that supports elementwise mathematical operations

In this version, the Vector call inherits from list; this 
requires Python 2.2 or later.
"""

class Vector(list):
	"""
	A list based Vector class
	"""
	# no c'tor
	
	def __getslice__(self, i, j):
		try:
			# use the list __getslice__ method and convert
			# result to Vector
			return Vector(super(Vector, self).__getslice__(i,j))
		except:
			raise TypeError, 'Vector::FAILURE in __getslice__'
		
	def __add__(self, other):
		return Vector(map(lambda x,y: x+y, self, other))

	def __neg__(self):
		return Vector(map(lambda x: -x, self))
	
	def __sub__(self, other):
		return Vector(map(lambda x,y: x-y, self, other))

	def __mul__(self, other):
		"""
		Element by element multiplication
		"""
		try:
			return Vector(map(lambda x,y: x*y, self,other))
		except:
		    # other is a const
		    return Vector(map(lambda x: x*other, self))


	def __rmul__(self, other):
		return (self*other)


	def __div__(self, other):
		"""
		Element by element division.
		"""
		try:
			return Vector(map(lambda x,y: x/y, self, other))
		except:
		    return Vector(map(lambda x: x/other, self))

	def __rdiv__(self, other):
		"""
		The same as __div__
		"""
		try:
		    return Vector(map(lambda x,y: x/y, other, self))
		except:
		    # other is a const
		    return Vector(map(lambda x: other/x, self))

	def size(self): return len(self)

	def conjugate(self):
	    return Vector(map(lambda x: x.conjugate(), self))

	def ReIm(self):
		"""
		Return the real and imaginary parts
		"""
		return [
			Vector(map(lambda x: x.real, self)),
			Vector(map(lambda x: x.imag, self)),
			]
	
	def AbsArg(self):
		"""
		Return modulus and phase parts
		"""
		return [
			Vector(map(lambda x: abs(x), self)),
			Vector(map(lambda x: math.atan2(x.imag,x.real), self)),
			]


	def out(self):
	    """
	    Prints out the Vector.
	    """
	    print self

###############################################################################


def isVector(x):
    """
    Determines if the argument is a Vector class object.
    """
    return hasattr(x,'__class__') and x.__class__ is Vector

def zeros(n):
    """
    Returns a zero Vector of length n.
    """
    return Vector(map(lambda x: 0., range(n)))

def ones(n):
    """
    Returns a Vector of length n with all ones.
    """
    return Vector(map(lambda x: 1., range(n)))

def random(n, lmin=0.0, lmax=1.0):
	"""
	Returns a random Vector of length n.
	"""
	import whrandom
	new = Vector([])
	gen = whrandom.whrandom()
	dl = lmax-lmin
	return Vector(map(lambda x: dl*gen.random(),
		range(n)))
	
def dot(a, b):
	"""
	dot product of two Vectors.
	"""
	try:
		return reduce(lambda x, y: x+y, a*b, 0.)
	except:
		raise TypeError, 'Vector::FAILURE in dot'
	
def cross3d(v1,v2):
	"""
	cross product of two Vectors.
	"""
	try:
		#(1 2 - 2 1) i + (2 0 - 0 2) j + (0 1 - 1 0) k
		return Vector([ (v1[1]*v2[2] - v1[2]*v2[1]), 
				 (v1[2]*v2[0] - v1[0]*v2[2]),
				 (v1[0]*v2[1] - v1[1]*v2[0])
				])
	except:
		raise TypeError, 'Vector::FAILURE in cross'


def norm(a):
	"""
	Computes the norm of Vector a.
	"""
	try:
		return math.sqrt(abs(dot(a,a)))
	except:
		raise TypeError, 'Vector::FAILURE in norm'

def sum(a):
	"""
	Returns the sum of the elements of a.
	"""
	try:
		return reduce(lambda x, y: x+y, a, 0)
	except:
		raise TypeError, 'Vector::FAILURE in sum'

# elementwise operations
	
def log10(a):
	"""
	log10 of each element of a.
	"""
	try:
		return Vector(map(math.log10, a))
	except:
		raise TypeError, 'Vector::FAILURE in log10'

def log(a):
	"""
	log of each element of a.
	"""
	try:
		return Vector(map(math.log, a))
	except:
		raise TypeError, 'Vector::FAILURE in log'

def exp(a):
	"""
	Elementwise exponential.
	"""
	try:
		return Vector(map(math.exp, a))
	except:
		raise TypeError, 'Vector::FAILURE in exp'

def sin(a):
	"""
	Elementwise sine.
	"""
	try:
		return Vector(map(math.sin, a))
	except:
		raise TypeError, 'Vector::FAILURE in sin'

def tan(a):
	"""
	Elementwise tangent.
	"""
	try:
		return Vector(map(math.tan, a))
	except:
		raise TypeError, 'Vector::FAILURE in tan'
	
def cos(a):
	"""
	Elementwise cosine.
	"""
	try:
		return Vector(map(math.cos, a))
	except:
		raise TypeError, 'Vector::FAILURE in cos'

def asin(a):
	"""
	Elementwise inverse sine.
	"""
	try:
		return Vector(map(math.asin, a))
	except:
		raise TypeError, 'Vector::FAILURE in asin'

def atan(a):
	"""
	Elementwise inverse tangent.
	"""	
	try:
		return Vector(map(math.atan, a))
	except:
		raise TypeError, 'Vector::FAILURE in atan'

def acos(a):
	"""
	Elementwise inverse cosine.
	"""
	try:
		return Vector(map(math.acos, a))
	except:
		raise TypeError, 'Vector::FAILURE in acos'

def sqrt(a):
	"""
	Elementwise sqrt.
	"""
	try:
		return Vector(map(math.sqrt, a))
	except:
		raise TypeError, 'Vector::FAILURE in sqrt'

def sinh(a):
	"""
	Elementwise hyperbolic sine.
	"""
	try:
		return Vector(map(math.sinh, a))
	except:
		raise TypeError, 'Vector::FAILURE in sinh'

def tanh(a):
	"""
	Elementwise hyperbolic tangent.
	"""
	try:
		return Vector(map(math.tanh, a))
	except:
		raise TypeError, 'Vector::FAILURE in tanh'

def cosh(a):
	"""
	Elementwise hyperbolic cosine.
	"""
	try:
		return Vector(map(math.cosh, a))
	except:
		raise TypeError, 'Vector::FAILURE in cosh'


def pow(a,b):
	"""
	Takes the elements of a and raises them to the b-th power
	"""
	try:
		return Vector(map(lambda x: x**b, a))
	except:
		try:
			return Vector(map(lambda x,y: x**y, a, b))
		except:
			raise TypeError, 'Vector::FAILURE in pow'
	
def atan2(a,b):    
	"""
	Arc tangent
	"""
	try:
		return Vector(map(math.atan2, a, b))
	except:
		raise TypeError, 'Vector::FAILURE in atan2'
	

###############################################################################
if __name__ == "__main__":

	print 'a = zeros(4)'
	a = zeros(4)

	print 'a.__doc__=',a.__doc__

	print 'a[0] = 1.0'
	a[0] = 1.0

	print 'a[3] = 3.0'
	a[3] = 3.0

	print 'a[0]=', a[0]
	print 'a[1]=', a[1]

	print 'len(a)=',len(a)
	print 'a.size()=', a.size()
			
	b = Vector([1, 2, 3, 4])
	print 'a=', a
	print 'b=', b

	print 'a+b'
	c = a + b
	c.out()

	print '-a'
	c = -a
	c.out()
	a.out()

	print 'a-b'
	c = a - b
	c.out()

	print 'a*1.2'
	c = a*1.2
	c.out()


	print '1.2*a'
	c = 1.2*a
	c.out()
	print 'a=', a

	print 'dot(a,b) = ', dot(a,b)
	print 'dot(b,a) = ', dot(b,a)

	print 'a*b'
	c = a*b
	c.out()
	
	print 'a/1.2'
	c = a/1.2
	c.out()

	print 'a[0:2]'
	c = a[0:2]
	c.out()

	print 'a[2:5] = [9.0, 4.0, 5.0]'
	a[2:5] = [9.0, 4.0, 5.0]
	a.out()

	print 'sqrt(a)=',sqrt(a)
	print 'pow(a, 2*ones(len(a)))=',pow(a, 2*ones(len(a)))
	print 'pow(a, 2)=',pow(a, 2*ones(len(a)))

	print 'ones(10)'
	c = ones(10)
	c.out()

	print 'zeros(10)'
	c = zeros(10)
	c.out()	

	print 'del a'
	del a

	try:
		a = random(11, 0., 2.)
		a.out()

	except: pass

	a = Vector([3, -3, 1])
	b = Vector([4,9,2])
	
	a = Vector([3, -2, -2])
	b = Vector([-1,0,5])

	print 'Cross(a,b)',cross3d(a,b)
