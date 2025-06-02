function varargout = Create3SimulationToolboxVer
% CREATE3SIMULATIONTOOLBOXVER displays the Create3 SimulationToolbox for  
% MATLAB version information.
%   CREATE3SIMULATIONTOOLBOXVER displays the version information to the 
%   command prompt.
%
%   A = CREATE3SIMULATIONTOOLBOXVER returns in A the sorted struct array of  
%   version information for the Create3 Simulation Toolbox.
%     The definition of struct A is:
%             A.Name      : toolbox name
%             A.Version   : toolbox version number
%             A.Release   : toolbox release string
%             A.Date      : toolbox release date
%
%   L. DeVries & M. Kutzer, 31Oct2024, USNA

% Updates:
%   06Nov2024 - Labeled transform objects for world and body frame
%   06Nov2024 - Include occupancy map support
%   22May2025 - Updated for local user install

A.Name = 'Create3 Simulation Toolbox';
A.Version = '1.0.3';
A.Release = '(R2023a)';
A.Date = '22-May-2025';
A.URLVer = 1;

msg{1} = sprintf('MATLAB %s Version: %s %s',A.Name, A.Version, A.Release);
msg{2} = sprintf('Release Date: %s',A.Date);

n = 0;
for i = 1:numel(msg)
    n = max( [n,numel(msg{i})] );
end

fprintf('%s\n',repmat('-',1,n));
for i = 1:numel(msg)
    fprintf('%s\n',msg{i});
end
fprintf('%s\n',repmat('-',1,n));

if nargout == 1
    varargout{1} = A;
end