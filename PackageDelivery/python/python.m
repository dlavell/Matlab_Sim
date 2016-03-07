function [result, status] = python(varargin)
%PYTHON Execute Python command and return the result.
%   PYTHON(PYTHONFILE) calls perl script specified by the file PYTHONFILE
%   using appropriate perl executable.
%
%   PYTHON(PYTHONFILE,ARG1,ARG2,...) passes the arguments ARG1,ARG2,...
%   to the perl script file PYTHONFILE, and calls it by using appropriate
%   perl executable.
%
%   RESULT=PYTHON(...) outputs the result of attempted perl call.  If the
%   exit status of perl is not zero, an error will be returned.
%
%   [RESULT,STATUS] = PYTHON(...) outputs the result of the perl call, and
%   also saves its exit status into variable STATUS.
%
%   If the Python executable is not available, it can be downloaded from:
%     https://www.python.org/
%
%   See also SYSTEM, JAVA, MEX.

%   Adapted from perl.m located in:
%   C:\Program Files\MATLAB\R2014b\toolbox\matlab\general\perl.m
%   Note: The command error() checking and return might not work because they are
%   using the perl MATLAB error messages

%   Test cmd: python('C:\Users\dlavell\AppData\Local\Programs\Python\Python35-32\Tools\demo\beer.py')

%   This is the hardcoded path to the python.exe
pythonPath = fullfile('./python_library/python.exe');

cmdString = '';

% Add input to arguments to operating system command to be executed.
% (If an argument refers to a file on the MATLAB path, use full file path.)
for i = 1:nargin
    thisArg = varargin{i};
    if ~ischar(thisArg)
        error(message('MATLAB:python:InputsMustBeStrings'));
    end
    if i==1
        if exist(thisArg, 'file')==2
            % This is a valid file on the MATLAB path
            if isempty(dir(thisArg))
                % Not complete file specification
                % - file is not in current directory
                % - OR filename specified without extension
                % ==> get full file path
                thisArg = which(thisArg);
            end
        else
            % First input argument is PythonFile - it must be a valid file
            error(message('MATLAB:python:FileNotFound', thisArg));
        end
    end

    % Wrap thisArg in double quotes if it contains spaces
    if isempty(thisArg) || any(thisArg == ' ')
        thisArg = ['"', thisArg, '"']; %#ok<AGROW>
    end

    % Add argument to command string
    cmdString = [cmdString, ' ', thisArg]; %#ok<AGROW>
end

% Execute Python script
if isempty(cmdString)
    error(message('MATLAB:perl:NoPerlCommand'));
elseif ispc
    % PC
    %pythonCmd = fullfile(matlabroot, pythonPath);
    %cmdString = ['python' cmdString];
    %pythonCmd = ['set PATH=',pythonCmd, ';%PATH%&' cmdString];
    pythonCmd = strcat(pythonPath, cmdString);
    [status, result] = dos(pythonCmd);
else
    % UNIX
    [status, ~] = unix('which python');
    if (status == 0)
        cmdString = ['python', cmdString];
        [status, result] = unix(cmdString);
    else
        error(message('MATLAB:perl:NoExecutable'));
    end
end

% Check for errors in shell command
if nargout < 2 && status~=0
    error(message('MATLAB:perl:ExecutionError', result, cmdString));
end

