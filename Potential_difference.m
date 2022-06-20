function varargout = Potential_difference(varargin)
% POTENTIAL_DIFFERENCE MATLAB code for Potential_difference.fig
%      POTENTIAL_DIFFERENCE, by itself, creates a new POTENTIAL_DIFFERENCE or raises the existing
%      singleton*.
%
%      H = POTENTIAL_DIFFERENCE returns the handle to a new POTENTIAL_DIFFERENCE or the handle to
%      the existing singleton*.
%
%      POTENTIAL_DIFFERENCE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in POTENTIAL_DIFFERENCE.M with the given input arguments.
%
%      POTENTIAL_DIFFERENCE('Property','Value',...) creates a new POTENTIAL_DIFFERENCE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Potential_difference_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Potential_difference_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES
%
% Author: Zeynep Akalin Acar, SCCN, 2019

% Copyright (C) 2007 Zeynep Akalin Acar, SCCN, zeynep@sccn.ucsd.edu
%
% This program is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 2 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program; if not, write to the Free Software
% Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


% Edit the above text to modify the response to help Potential_difference

% Last Modified by GUIDE v2.5 04-Jun-2019 21:09:37

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Potential_difference_OpeningFcn, ...
                   'gui_OutputFcn',  @Potential_difference_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Potential_difference is made visible.
function Potential_difference_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Potential_difference (see VARARGIN)

% Choose default command line output for Potential_difference
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Potential_difference wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Potential_difference_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function handles = load_mesh_and_sensors(hObject, handles)

if isfield(handles, 'coord')
    return
end

[Coord,Elem] = ReadSMF('Scalp.smf',0,0,0,1); 
sens = load('se_s1.sensors','-mat');
snormal = sens.pnt-mean(Coord(:,2:4)); % Compute sensor 'direction'
snormal = snormal./vecnorm(snormal,2,2);
handles.coord = Coord(:,2:4);
handles.elem = Elem(:,2:4);
handles.sens = sens.pnt; % save sens for future use
handles.snormal = snormal; % save it for future use
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2

% 1st plot

handles = load_mesh_and_sensors(hObject, handles);
delete(get(gca,'Children'));

plotmesh(handles.elem,handles.coord,[],1);
view(120,15)
hold on
rotate3d on

kd = 4:131;
plot3(handles.sens(kd,1),handles.sens(kd,2),handles.sens(kd,3),'r.','markersize',20)

Pn = handles.sens+handles.snormal * 15;

for i = kd
    text(Pn(i,1),Pn(i,2),Pn(i,3),sprintf('%d',i))
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in pushbutton1.
conf = nft_get_config;
of = pwd;


Iin = str2num(get(handles.edit1,'String'));
Iout = str2num(get(handles.edit2,'String'));
if Iin == Inf; Iin = [4:131]; end;
if Iout == Inf; Iout = [4:131]; end;

load Model_LFM LFM

V1 = mean(LFM(Iin,:),1);
V2 = mean(LFM(Iout,:),1);

pot = V1-V2;

sIn = handles.sens(Iin,:);
sOut = handles.sens(Iout,:);

nIn = -1 * handles.snormal(Iin,:);
nOut = handles.snormal(Iout,:);

save pot pot -ascii

% curr_dir = pwd;
% of = handles.MeshFolder;
% cd(of)
[az, el] = view(handles.axes3);


f = fopen(sprintf('%s/StepSc.txt',of), 'w');
fprintf(f, 'set interp 1\n');
fprintf(f, 'nfield load %s/pot\n',of);
fprintf(f, 'set view %d 0 %d\n', el-90, -az);
d = 1;
for i = 1:size(sIn,1)
    fprintf(f, 'set dipole %d %g %g %g %g %g %g\n', d, sIn(i,1), sIn(i,2), sIn(i,3), nIn(i,1), nIn(i,2), nIn(i,3));
    fprintf(f, 'set dcolor %d 1 0 0\n',d);
    fprintf(f, 'set dscale %d 1.0\n',d);
    d = d + 1;
end
for i = 1:size(sOut,1)
    fprintf(f, 'set dipole %d %g %g %g %g %g %g\n', d, sOut(i,1), sOut(i,2), sOut(i,3), nOut(i,1), nOut(i,2), nOut(i,3));
    fprintf(f, 'set dcolor %d 0 0 1\n',d);
    fprintf(f, 'set dscale %d 1.0\n',d);
    d = d + 1;
end

%fprintf(f, 'quit\n');
fclose(f);
f = helpdlg('Running external viewer. Close Showmesh to continue ...');
a = sprintf('"%s" -c "%s/StepSc.txt" FSss.smf', conf.showmesh3, of);
[status, result] = system(a);
if status ~= 0
    error('Mesh_Generation:system','Failed to execute: %s',result);
end
if ishandle(f)
    delete(f);
end

% cd(curr_dir)


% --- Executes during object creation, after setting all properties.
function axes4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes4
handles = load_mesh_and_sensors(hObject, handles);
delete(get(gca,'Children'));
plotmesh(handles.elem,handles.coord,[],1);
view(120,15)
hold on
rotate3d on




% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


axes(handles.axes4); % Make axes 4 the current axes
%delete(get(gca,'Children')); % Clear contents

[az,el] = view;
axes4_CreateFcn(handles.axes4, eventdata, handles)
view(az,el);

Pn = handles.sens+handles.snormal * 15;

Iin = str2num(get(handles.edit1,'String'));
Iout = str2num(get(handles.edit2,'String'));
if Iin == Inf; Iin = [4:131]; end;
if Iout == Inf; Iout = [4:131]; end;

kd = Iout;
plot3(handles.sens(kd,1),handles.sens(kd,2),handles.sens(kd,3),'b.','markersize',20)

for i = kd
    text(Pn(i,1),Pn(i,2),Pn(i,3),sprintf('%d',i))
end

kd = Iin;
plot3(handles.sens(kd,1),handles.sens(kd,2),handles.sens(kd,3),'r.','markersize',20)

for i = kd
    text(Pn(i,1),Pn(i,2),Pn(i,3),sprintf('%d',i))
end


% --- Executes during object creation, after setting all properties.
function uipanel2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uipanel2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
