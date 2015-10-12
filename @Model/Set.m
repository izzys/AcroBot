function [ Mod ] = Set( Mod, varargin )
% Sets desired object properties
% Use as Mod.Set('m',3,'mh',10,...)

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
            case 'm' % leg mass
                k = Mod.m_radius/Mod.m;
                Mod.m = value;
                Mod.m_radius=k*Mod.m;
            case 'L' % leg length
                Mod.L = value;
            case 'I' % leg moment of inertia
                Mod.I = value;
            case 'g' % gravity
                Mod.g = value;
            case 'damping' 
                Mod.damping = value;
     
            %  Render parameters:
            case 'm_radius'
                Mod.m_radius = value*Mod.m;
            case 'm_color'
                Mod.m_color = value;
            case 'leg_width'
                Mod.leg_width = value;
            case 'leg_color'
                Mod.leg_color = value;
            case 'CircRes'
                Mod.CircRes = value;
            case 'LinkRes'
                Mod.LinkRes = value;
            case 'LineWidth'
                Mod.LineWidth = value;
            otherwise
                error(['Set failed: ',key,' property not found']);
        end

    end
end

