function [us,t,mags,phs]=schroed(T,Ns,relmag,usat)
%
% This routine computes a Schroeder phased time domain signal 
%
%              Ns/2
%      us(k+1) = sum    mags(i)*cos(w(i)*k*T+phs(i)), k=0,...,Ns-1
%              i=1
% where 
%       w(i)=2*pi*i/(Ns*T), i=1,...,Ns/2
%       mags- magnitudes  proportional to relmag (specified by user)
%       phs - Schroeder phases 
%
% ALL VECTORS ARE ROW VECTORS!
% INPUTS: 
% T      - sampling period (secs)
% Ns     - number of samples in one period of Schroeder
%          signal; also number of FFT points used in 
%          spectral estimation. Note that Ns/2 is number
%          of sinusoids in Schroeder sum above.  
% relmag - Ns/2 vector of relative sinusoidal magnitudes 
%          (e.g., relmag=ones(1:Ns/2) for flat spectrum)
% usat   - saturation limit on Schroeder signal
%
% OUTPUTS:
% us      - one period (Ns*T seconds) of Schroeder signal 
% t       - time axis for us -- as in plot(t,us)
% mags    - vector of Schroeder magnitudes 
%           (proportional to relmag)
% phs     - vector of Schroeder phases 
%
% Written by D.S. Bayard, 6/91
% Copyright (C) California Institute of Technology, 1991
%
% form alpha and normalize
alpha=relmag.^2/2;
ss=sum(abs(alpha));
alphn=alpha/ss;
tpi=2*pi;
jj=sqrt(-1);
Tp=Ns*T;
t=T*[0:1:Ns-1];
ns=Ns/2;
w=2*pi*[1:ns]/Tp;
j=1:1:ns;
% construct Schroeder phases phs
phs(1)=tpi*alphn(1);
for i=1:1:ns-1,
phs(i+1)=tpi*(i+1)*alphn(i+1)+phs(i);
end
% set up Schroeder signal in complex domain
Us1=sqrt(2*alpha).*exp(jj*phs);
% stuff DC with zero for using Ns point fft
Us=[0,Us1];
% construct standard time signal uss
uscmplx=ifft(Us,Ns)*Ns;
uss=real(uscmplx);
%usq=imag(uscmplx);
% uss is sum of cosines with 
% phase=phs and mag=sqrt(2*alpha)
% normalize to form input us
mus=max(abs(uss));
us=usat*uss/mus;
mags=sqrt(2*alpha)*usat/mus;

