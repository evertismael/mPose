import numpy as np
import pylab as pb
from scipy.special import gammaln, digamma
from utilities import pdinv

class prior:
	def pdf(self,x):
		return np.exp(self.lnpdf(x))
	def plot(self):
		rvs = self.rvs(1000)
		pb.hist(rvs,100,normed=True)
		xmin,xmax = pb.xlim()
		xx = np.linspace(xmin,xmax,1000)
		pb.plot(xx,self.pdf(xx),'r',linewidth=2)

class Gaussian(prior):
	"""
	Implementation of the univariate Gaussian probability function, coupled with random variables, since scipy.stats sucks.
	Using Bishop 2006 notation"""
	def __init__(self,mu,sigma):
		self.mu = float(mu)
		self.sigma = float(sigma)
		self.sigma2 = np.square(self.sigma)
		self.constant = -0.5*np.log(2*np.pi*self.sigma2)
	def summary(self):
		pass #TODO
	def lnpdf(self,x):
		return self.constant - 0.5*np.square(x-self.mu)/self.sigma2
	def lnpdf_grad(self,x):
		return -(x-self.mu)/self.sigma2
	def rvs(self,n):
		return np.random.randn(n)*self.sigma + self.mu

class log_Gaussian(prior):
	"""
	"""
	def __init__(self,mu,sigma):
		self.mu = float(mu)
		self.sigma = float(sigma)
		self.sigma2 = np.square(self.sigma)
		self.constant = -0.5*np.log(2*np.pi*self.sigma2)
	def summary(self):
		pass #TODO
	def lnpdf(self,x):
		return self.constant - 0.5*np.square(np.log(x)-self.mu)/self.sigma2 -np.log(x)
	def lnpdf_grad(self,x):
		return -((np.log(x)-self.mu)/self.sigma2+1.)/x
	def rvs(self,n):
		return np.exp(np.random.randn(n)*self.sigma + self.mu)

class multivariate_Gaussian:
	"""
	Implementation of the multivariate Gaussian probability function, coupled with random variables, since scipy.stats sucks.
	Using Bishop 2006 notation"""
	def __init__(self,mu,var):
		self.mu = np.array(mu).flatten()
		self.var = np.array(var)
		assert len(self.var.shape)==2
		assert self.var.shape[0]==self.var.shape[1]
		assert self.var.shape[0]==self.mu.size
		self.D = self.mu.size
		self.inv, self.hld = pdinv(self.var)
		self.constant = -0.5*self.D*np.log(2*np.pi) - self.hld

	def summary(self):
		pass #TODO
	def pdf(self,x):
		return np.exp(self.lnpdf(x))
	def lnpdf(self,x):
		d = x-self.mu
		return self.constant - 0.5*np.sum(d*np.dot(d,self.inv),1)
	def lnpdf_grad(self,x):
		d = x-self.mu
		return -np.dot(self.inv,d)
	def rvs(self,n):
		return np.random.multivariate_normal(self.mu, self.var,n)
	def plot(self):
		if self.D==2:
			rvs = self.rvs(200)
			pb.plot(rvs[:,0],rvs[:,1], 'kx', mew=1.5)
			xmin,xmax = pb.xlim()
			ymin,ymax = pb.ylim()
			xx, yy = np.mgrid[xmin:xmax:100j, ymin:ymax:100j]
			xflat = np.vstack((xx.flatten(),yy.flatten())).T
			zz = self.pdf(xflat).reshape(100,100)
			pb.contour(xx,yy,zz,linewidths=2)


def gamma_from_EV(E,V):
	"""create an instance of a gamma prior  by specifying the Expected value(s) and Variance(s) of the distribution"""
	a = np.square(E)/V
	b = E/V
	if np.size(a)>1:
		return multiple_gamma(a,b)
	else:
		return gamma(a,b)


class gamma(prior):
	"""
	Implementation of the Gamma probability function, coupled with random variables, since scipy.stats sucks.
	Using Bishop 2006 notation
	"""
	def __init__(self,a,b):
		self.a = float(a)
		self.b = float(b)
		self.constant = -gammaln(self.a) + a*np.log(b)

	def summary(self):
		ret = {"E[x]": self.a/self.b,\
			"E[ln x]": digamma(self.a) - np.log(self.b),\
			"var[x]": self.a/self.b/self.b,\
			"Entropy": gammaln(self.a) - (self.a-1.)*digamma(self.a) - np.log(self.b) + self.a}
		if self.a >1:
			ret['Mode'] = (self.a-1.)/self.b
		else:
			ret['mode'] = np.nan
		return ret
	def lnpdf(self,x):
		return self.constant + (self.a-1)*np.log(x) - self.b*x
	def lnpdf_grad(self,x):
		return (self.a-1.)/x - self.b
	def rvs(self,n):
		return np.random.gamma(scale=1./self.b,shape=self.a,size=n)

