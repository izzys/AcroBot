function [ Con ] = Set( Con, varargin )
% Sets desired object properties

nParams = (nargin-1)/2;
if rem(nParams,1)~=0 || nargin<1
    error('Set failed: not enough inputs')
else
    for p = 1:nParams
        key = varargin{2*p-1};
        value = varargin{2*p};
        if ~isnumeric(value)
            error('Set failed: property value must be numeric');
        end
        
        switch key

            case 'omega0'
                Con.omega0 = value;
            case 'Gain'
                Con.Gain = value; 
            case 'Pulse_Width'
                Con.Pulse_Width = value; 
            case 'gamma'
                Con.gamma = value; 
            case 'mu'
                Con.mu = value; 
            case 'NumOfNeurons'
                Con.NumOfNeurons = value; 
            case'epsilon'
                Con.epsilon = value; 
            case 'tau'
                Con.tau = value; 
            case 'eta'
                Con.eta = value; 
            case 'Amp_teach'
                Con.Amp_teach = value; 
            case 'Omega_teach'
                Con.Omega_teach = value;
            case 'Phi_teach'
                Con.Phi_teach = value; 
            otherwise
                error(['Set failed: ',key,' property not found']);
        end
    end
end

