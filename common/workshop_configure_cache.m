function workshop_configure_cache()
%WORKSHOP_CONFIGURE_CACHE Keep generated files separate for each MATLAB release.
%   Accelerator builds and simulation caches cannot be shared across releases.
%   Derive the project root from this file so the working folder does not matter.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
releaseFolder = fullfile(projectRoot, 'cache', ['R' version('-release')]);
Simulink.fileGenControl('set', ...
    'CacheFolder', releaseFolder, ...
    'CodeGenFolder', releaseFolder, ...
    'createDir', true);
end
