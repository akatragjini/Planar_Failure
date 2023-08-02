function varargout = RPlane(varargin)
% RPLANE MATLAB code for RPlane.fig
%      RPLANE, by itself, creates a new RPLANE or raises the existing
%      singleton*.
%
%      H = RPLANE returns the handle to a new RPLANE or the handle to
%      the existing singleton*.
%
%      RPLANE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RPLANE.M with the given input arguments.
%
%      RPLANE('Property','Value',...) creates a new RPLANE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before RPlane_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to RPlane_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help RPlane

% Last Modified by GUIDE v2.5 06-Mar-2015 18:28:54

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @RPlane_OpeningFcn, ...
                   'gui_OutputFcn',  @RPlane_OutputFcn, ...
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


% --- Executes just before RPlane is made visible.
function RPlane_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to RPlane (see VARARGIN)

% Choose default command line output for RPlane
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes RPlane wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = RPlane_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function Iterations_Callback(hObject, eventdata, handles)
% hObject    handle to Iterations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Iterations as text
%        str2double(get(hObject,'String')) returns contents of Iterations as a double


% --- Executes during object creation, after setting all properties.
function Iterations_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Iterations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Slopeht_Callback(hObject, eventdata, handles)
% hObject    handle to Slopeht (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Slopeht as text
%        str2double(get(hObject,'String')) returns contents of Slopeht as a double


% --- Executes during object creation, after setting all properties.
function Slopeht_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slopeht (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function DaylHt_Callback(hObject, eventdata, handles)
% hObject    handle to DaylHt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of DaylHt as text
%        str2double(get(hObject,'String')) returns contents of DaylHt as a double


% --- Executes during object creation, after setting all properties.
function DaylHt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DaylHt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Slopeangle_Callback(hObject, eventdata, handles)
% hObject    handle to Slopeangle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Slopeangle as text
%        str2double(get(hObject,'String')) returns contents of Slopeangle as a double


% --- Executes during object creation, after setting all properties.
function Slopeangle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slopeangle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function JointAngle_Callback(hObject, eventdata, handles)
% hObject    handle to JointAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of JointAngle as text
%        str2double(get(hObject,'String')) returns contents of JointAngle as a double


% --- Executes during object creation, after setting all properties.
function JointAngle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to JointAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function TensionDepth_u_Callback(hObject, eventdata, handles)
% hObject    handle to TensionDepth_u (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TensionDepth_u as text
%        str2double(get(hObject,'String')) returns contents of TensionDepth_u as a double


% --- Executes during object creation, after setting all properties.
function TensionDepth_u_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TensionDepth_u (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Tensiondepth_std_Callback(hObject, eventdata, handles)
% hObject    handle to Tensiondepth_std (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Tensiondepth_std as text
%        str2double(get(hObject,'String')) returns contents of Tensiondepth_std as a double


% --- Executes during object creation, after setting all properties.
function Tensiondepth_std_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Tensiondepth_std (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cohesion_u_Callback(hObject, eventdata, handles)
% hObject    handle to Cohesion_u (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cohesion_u as text
%        str2double(get(hObject,'String')) returns contents of Cohesion_u as a double


% --- Executes during object creation, after setting all properties.
function Cohesion_u_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cohesion_u (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cohesion_Std_Callback(hObject, eventdata, handles)
% hObject    handle to Cohesion_Std (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cohesion_Std as text
%        str2double(get(hObject,'String')) returns contents of Cohesion_Std as a double


% --- Executes during object creation, after setting all properties.
function Cohesion_Std_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cohesion_Std (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Fricangl_u_Callback(hObject, eventdata, handles)
% hObject    handle to Fricangl_u (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Fricangl_u as text
%        str2double(get(hObject,'String')) returns contents of Fricangl_u as a double


% --- Executes during object creation, after setting all properties.
function Fricangl_u_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Fricangl_u (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function fricangl_std_Callback(hObject, eventdata, handles)
% hObject    handle to fricangl_std (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fricangl_std as text
%        str2double(get(hObject,'String')) returns contents of fricangl_std as a double


% --- Executes during object creation, after setting all properties.
function fricangl_std_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fricangl_std (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Density_u_Callback(hObject, eventdata, handles)
% hObject    handle to Density_u (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Density_u as text
%        str2double(get(hObject,'String')) returns contents of Density_u as a double


% --- Executes during object creation, after setting all properties.
function Density_u_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Density_u (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Density_std_Callback(hObject, eventdata, handles)
% hObject    handle to Density_std (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Density_std as text
%        str2double(get(hObject,'String')) returns contents of Density_std as a double


% --- Executes during object creation, after setting all properties.
function Density_std_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Density_std (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function gravity_Callback(hObject, eventdata, handles)
% hObject    handle to gravity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of gravity as text
%        str2double(get(hObject,'String')) returns contents of gravity as a double


% --- Executes during object creation, after setting all properties.
function gravity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gravity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Load_Callback(hObject, eventdata, handles)
% hObject    handle to Load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Load as text
%        str2double(get(hObject,'String')) returns contents of Load as a double


% --- Executes during object creation, after setting all properties.
function Load_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function UnitWT_Callback(hObject, eventdata, handles)
% hObject    handle to UnitWT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of UnitWT as text
%        str2double(get(hObject,'String')) returns contents of UnitWT as a double


% --- Executes during object creation, after setting all properties.
function UnitWT_CreateFcn(hObject, eventdata, handles)
% hObject    handle to UnitWT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PGA_Callback(hObject, eventdata, handles)
% hObject    handle to PGA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PGA as text
%        str2double(get(hObject,'String')) returns contents of PGA as a double


% --- Executes during object creation, after setting all properties.
function PGA_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PGA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in ComputeFOS.
function ComputeFOS_Callback(hObject, eventdata, handles)
% hObject    handle to ComputeFOS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.FOSMatrixA = [];
handles.FdrMatrixA = [];
handles.FreMatrixA = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%RETREIVE INPUTS%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i  = str2num(get(handles.Iterations,'String')); %Number of iterations
H  = str2num(get(handles.Slopeht,'String'));    %Vertical height of slope (m)
H1 = str2num(get(handles.DaylHt,'String'));     %Vertical height to toe of discontinuity (m)
a  = str2num(get(handles.Slopeangle,'String')); %Slope angle, clockwise (a<90) (degrees)
t  = str2num(get(handles.JointAngle,'String')); %Discontinuity angle, clockwise (t<90 & t<a) (degrees)

vD    = str2num(get(handles.TensionDepth_u,'String'));          %Mean Tension Crack Depth (m)
vDs   = str2num(get(handles.Tensiondepth_std,'String'));        %Std. Dev. Tension Crack Depth (m)
vc    = (str2num(get(handles.Cohesion_u,'String')))/(1*10^-6);  %Mean Cohesion (Pa)
vcs   = (str2num(get(handles.Cohesion_Std,'String')))/(1*10^-6);%Std. Dev. Cohesion (Pa) 
vphi  = str2num(get(handles.Fricangl_u,'String'));              %Mean Friction Angle (degrees)
vphis = str2num(get(handles.fricangl_std,'String'));            %Std. Dev. Friction Angle (degrees)
vp    = str2num(get(handles.Density_u,'String'));               %Mean Density (kg/m3)
vps   = str2num(get(handles.Density_std,'String'));             %Std. Dev. Density (kg/m3)

q  = str2num(get(handles.PGA,'String'));     %http://en.wikipedia.org/wiki/Peak_ground_acceleration (unitless)
g  = str2num(get(handles.gravity,'String')); %Gravitaty (m/s2)
Y  = str2num(get(handles.UnitWT,'String'));  %Unit weight of water (N/m3)
L0 = str2num(get(handles.Load,'String'));    %Load on top of slope. (N/m/m)

%Angle conversion
ar   = degtorad(a);
tr   = degtorad(t);
n0   = degtorad(90);
t0   = degtorad(180);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%Maximum & Critical Tension Crack Depth (Used in Development)%%%%%%
MD = ((((H/cos(n0-ar)) - (H1/sin(ar)))*(sin(ar-tr)))/(sin(n0+tr)));
CD = ((((H/cos(n0-ar)) - (H1/sin(ar)))*((cos(tr))*(sin(ar-tr)))));
%fprintf('Critical Tension Crack Depth = %.1f\n',CD)
%fprintf('Maximum  Tension Crack Depth = %.1f\n\n',MD)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


FOSMatrix = zeros(i,1);
FdrMatrix = zeros(i,1);
FreMatrix = zeros(i,1);
DMatrix   = zeros(i,1);
CMatrix   = zeros(i,1);
PhMatrix  = zeros(i,1);
PMatrix   = zeros(i,1);

%%%%%%%%%%%%%%%%%%%%%CALCULATING FACTORS OF SAFETY%%%%%%%%%%%%%%%%%%%%%
for j = 1:i
  
D   = vD + (vDs*(randn(1,1)));
c   = vc + (vcs*(randn(1,1)));
phi = vphi + (vphis*(randn(1,1)));
phir= degtorad(phi);
p   = vp + (vps*(randn(1,1)));

    if D > ((((H/cos(n0-ar)) - (H1/sin(ar)))*(sin(ar-tr)))/(sin(n0+tr)))

        %Tension Crack depth is not valid (too large)
        fprintf('Tension Crack Depth Not Valid\n\n')
        break  

    else %Tension Crack depth is valid

        if ((((H/cos(n0-ar)) - (H1/sin(ar)))*((cos(tr))*(sin(ar-tr)))) - D) >= 0
            %Is tension crack less than critical depth or at critical depth?

            if (((((H/cos(n0-ar)) - (H1/sin(ar)))*((cos(tr))*(sin(ar-tr)))) - D)) == 0

                %Tension crack at critical depth
                c11  = (H/cos(n0-ar)) - (H1/sin(ar));
                x10 = (H1/sin(ar));
                x11 = c11;
                c12 = (sin(ar-tr))*(c11);
                x13 = (D*tan(tr));
                x14 = (cos(ar-tr))*(c11);
                %fprintf('Critical Depth\n\n')
                Right=x11;
                Top=x13;
                Left=D;
                Bottom=x14;
                A11 = (D*x13)/2;
                A12 = (x12*x14)/2;
                A1  = A11+A12; %Area of Wedge

                Fd1 = (cos(tr)*Y*(D^2))/2;
                Fd2 = (sin(tr)*g*p*A1);
                Fd3 = (cos(tr)*q*g*p*A1);
                Fd4 = (sin(tr)*L0*(Top^2));
                Fd  = Fd1 + Fd2 + Fd3 + Fd4; %Units of N/m. Length into page

                Fr1 = c*Bottom;
                Fr2 = (cos(tr)*L0*Top);
                Fr3 = (cos(tr)*g*p*A1);
                Fr4 = (Bottom*Y*D)/2;
                Fr5 = (sin(tr)*Y*(D^2))/2;               
                Fr6 = (sin(tr)*q*g*p*A1);
                Fr7 = (Fr2 + Fr3 - Fr4 - Fr5 - Fr6)*tan(phir);
                Fr  = Fr1 + Fr7; %Units of N/m. Length into page

                FOS = Fr/Fd;

            else

                %Tension crack less than critical depth
                c01  = (H/cos(n0-ar)) - (H1/sin(ar));
                x00 = (H1/sin(ar));
                x01 = c01;
                x02 = (D*tan(tr));
                x03 = (D/cos(tr));
                x04 = ((sin(ar-tr))*c01) - (D/cos(tr));
                x05 = ((x04)/(sin(tr)));
                x06 = ((x04)/(tan(tr)));
                x07 = ((cos(ar-tr))*c01);
                x08 = c01*sin(ar-tr);
                %fprintf('Less than Critical Depth\n\n')
                Right=x01;
                Top=x02 + x05;
                Left=D;
                Bottom=x06 + x07;
                A01 = D*x05;
                A02 = (x06*x04)/2;
                A03 = (x02*D)/2;
                A04 = (x08*x07)/2;
                A0  = A01+A02+A03+A04; %Area of wedge

                Fd1 = (cos(tr)*Y*(D^2))/2;
                Fd2 = (sin(tr)*g*p*A0);
                Fd3 = (cos(tr)*q*g*p*A0);
                Fd4 = (sin(tr)*L0*(Top^2));
                Fd  = Fd1 + Fd2 + Fd3 + Fd4; % Units of N/m. Length into page
                
                Fr1 = c*Bottom;
                Fr2 = (cos(tr)*L0*Top);
                Fr3 = (cos(tr)*g*p*A0);
                Fr4 = (Bottom*Y*D)/2;
                Fr5 = (sin(tr)*Y*(D^2))/2;               
                Fr6 = (sin(tr)*q*g*p*A0);
                Fr7 = (Fr2 + Fr3 - Fr4 - Fr5 - Fr6)*tan(phir);
                Fr  = Fr1 + Fr7; %Units of N/m. Length into page

                FOS = Fr/Fd;

            end

        elseif ((((H/cos(n0-ar)) - (H1/sin(ar)))*((cos(tr))*(sin(ar-tr)))) - D) <=0
            %Is tension crack greater than, equal or at a maximum value of critical depth?

            if (abs((((H/cos(n0-ar)) - (H1/sin(ar)))*((cos(tr))*(sin(ar-tr)))) - D)) ==0

                %Tension crack at critical depth
                c11  = (H/cos(n0-ar)) - (H1/sin(ar));
                x10 = (H1/sin(ar));
                x11 = c11;
                c12 = (sin(ar-tr))*(c11);
                x13 = (D*tan(tr));
                x14 = (cos(ar-tr))*(c11);
                %fprintf('Critical Depth\n\n')
                Right=x11;
                Top=x13;
                Left=D;
                Bottom=x14;
                A11 = (D*x13)/2;
                A12 = (x12*x14)/2;
                A1  = A11+A12; %Area of wedge

                Fd1 = (cos(tr)*Y*(D^2))/2;
                Fd2 = (sin(tr)*g*p*A1);
                Fd3 = (cos(tr)*q*g*p*A1);
                Fd4 = (sin(tr)*L0*(Top^2));
                Fd  = Fd1 + Fd2 + Fd3 + Fd4; % Units of N/m. Length into page

                Fr1 = c*Bottom;
                Fr2 = (cos(tr)*L0*Top);
                Fr3 = (cos(tr)*g*p*A1);
                Fr4 = (Bottom*Y*D)/2;
                Fr5 = (sin(tr)*Y*(D^2))/2;               
                Fr6 = (sin(tr)*q*g*p*A1);
                Fr7 = (Fr2 + Fr3 - Fr4 - Fr5 - Fr6)*tan(phir);
                Fr  = Fr1 + Fr7; %Units of N/m. Length into page

                FOS = Fr/Fd;

            else % Tension crack greater than critical depth or at edge of slope?

                if (((((H/cos(n0-ar)) - (H1/sin(ar)))*(sin(ar-tr)))/(sin(n0+tr)))-D) ==0

                    %Tension crack at edge of slope (maximum depth)
                    c31 = (H/cos(n0-ar)) - (H1/sin(ar));
                    x30 = (H1/sin(ar));
                    x31 = c31;
                    x32 = ((c31*sin(n0-ar))/(sin(n0+tr)));  
                    %fprintf('Depth at Crest\n\n')
                    Right = x31;
                    Left = D;
                    Bottom = x32;
                    Top = 0;
                    A3 = ((((sin(n0-ar))*D) * ((cos(n0-ar))*D))/2) + ((((sin(ar-tr))*x32) * ((cos(ar-tr))*x32))/2);
                    %Area of Wedge

                    Fd1 = (cos(tr)*Y*(D^2))/2;
                    Fd2 = (sin(tr)*g*p*A3);
                    Fd3 = (cos(tr)*q*g*p*A3);
                    Fd4 = (sin(tr)*L0*(Top^2));
                    Fd  = Fd1 + Fd2 + Fd3 + Fd4; % Units of N/m. Length into page
                    
                    Fr1 = c*Bottom;
                    Fr2 = (cos(tr)*L0*Top);
                    Fr3 = (cos(tr)*g*p*A3);
                    Fr4 = (Bottom*Y*D)/2;
                    Fr5 = (sin(tr)*Y*(D^2))/2;               
                    Fr6 = (sin(tr)*q*g*p*A3);
                    Fr7 = (Fr2 + Fr3 - Fr4 - Fr5 - Fr6)*tan(phir);
                    Fr  = Fr1 + Fr7; %Units of N/m. Length into page

                    FOS = Fr/Fd;

                else

                    %Tension crack greater than critical depth
                    c21 = (H/cos(n0-ar)) - (H1/sin(ar));
                    x20 = (H1/sin(ar));
                    x21 = c21;
                    x22 = (D/sin(ar));
                    x23 = c21 - (x22);
                    x24 = (D/tan(ar));
                    x25 = (x23*(sin(ar-tr)))/(sin(tr));
                    x26 = (x23*sin(t0-ar))/(sin(tr));
                    x27 = (x25) - (x24);
                    %fprintf('Greater than Critical Depth\n\n')
                    Right= x21;
                    Top= x27;
                    Left=D;
                    Bottom=x26;
                    A21 = (D*x27);
                    A22 = (D*x24)/2;
                    A231 = (((sin(tr))*(x25)*(cos(tr))*(x25))/2);
                    A232 = (((sin(ar-tr))*(x23)*(cos(ar-tr))*(x23))/2);
                    A23 = A231+ A232;
                    A2  = A21 + A22 + A23; % Area of wedge

                    Fd1 = (cos(tr)*Y*(D^2))/2;
                    Fd2 = (sin(tr)*g*p*A2);
                    Fd3 = (cos(tr)*q*g*p*A2);
                    Fd4 = (sin(tr)*L0*(Top^2));
                    Fd  = Fd1 + Fd2 + Fd3 + Fd4; % Units of N/m. Length into page

                    Fr1 = c*Bottom;
                    Fr2 = (cos(tr)*L0*Top);
                    Fr3 = (cos(tr)*g*p*A2);
                    Fr4 = (Bottom*Y*D)/2;
                    Fr5 = (sin(tr)*Y*(D^2))/2;               
                    Fr6 = (sin(tr)*q*g*p*A2);
                    Fr7 = (Fr2 + Fr3 - Fr4 - Fr5 - Fr6)*tan(phir);
                    Fr  = Fr1 + Fr7; %Units of N/m. Length into page

                    FOS = Fr/Fd;

                end

            end

        end

    end
    
    FOSMatrix(j,1) = FOS;
    FdrMatrix(j,1) = Fd;
    FreMatrix(j,1) = Fr;
    DMatrix(j,1)   = D;
    CMatrix(j,1)   = c;
    PhMatrix(j,1)  = phi;
    PMatrix(j,1)   = p;
end

handles.FOSMatrixA = FOSMatrix;
handles.FdrMatrixA = FdrMatrix;
handles.FreMatrixA = FreMatrix;
handles.DMatrixA   = DMatrix;
handles.CMatrix    = CMatrix;
handles.PhMatrix   = PhMatrix;
handles.PMatrix    = PMatrix;

guidata(hObject,handles)

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
grid off


% --- Executes on button press in Clear.
function Clear_Callback(hObject, eventdata, handles)
% hObject    handle to Clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla,clc


% --- Executes on selection change in dropdownplots.
function dropdownplots_Callback(hObject, eventdata, handles)
% hObject    handle to dropdownplots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns dropdownplots contents as cell array
%        contents{get(hObject,'Value')} returns selected item from dropdownplots


% --- Executes during object creation, after setting all properties.
function dropdownplots_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dropdownplots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Plotbutton.
function Plotbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Plotbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

i  = str2num(get(handles.Iterations,'String')); %Number of iterations

string1 = get(handles.dropdownplots,'Value');
string2 = get(handles.dropdownplots,'String');
string  = string2(string1);

s0 = 'Factor of Safety (Bar)';
s1 = 'Cumulative Frequency';
s2 = 'Driving V.S. Resistive';
s3 = 'FOS & Wedge Depth';
s4 = 'FOS & Cohesion';
s5 = 'FOS & Friction Angle';
s6 = 'FOS & Density';

FOSMatrix = handles.FOSMatrixA;
FdrMatrix = handles.FdrMatrixA;
FreMatrix = handles.FreMatrixA;
DMatrix   = handles.DMatrixA;
CMatrix   = handles.CMatrix; 
PhMatrix  = handles.PhMatrix; 
PMatrix   = handles.PMatrix;

FOSa = sort(FOSMatrix,'ascend');
Index = (1:(i))/i;


if strcmp(string,s0)==1
    figure(1) % Histogram
    hist(FOSa,50,'Facecolor','k')
    grid on
    xlabel('Factor of Safety')
    ylabel('Number of Occurences')
    h = findobj(gca,'Type','patch');
    set(h,'FaceColor','k','EdgeColor','r','Linewidth',1)
elseif strcmp(string,s1)==1
    figure(2) %Cumulative Distribution
    plot(FOSa,Index,'k','Linewidth',2)
    grid on
    xlabel('Factor of Safety')
    ylabel('Cumulative Frequency')
elseif strcmp(string,s2)==1
    figure(3) % Driving Force & Resisting Force
    plot(FdrMatrix,FreMatrix,'k.','Markersize',5)
    grid on
    xlabel('Driving Force (N)')
    ylabel('Resisting Force (N)')
elseif strcmp(string,s3)==1
    figure(4) %Wedge Depth & FOS
    plot(DMatrix,FOSMatrix,'k.','Markersize',5)
    grid on
    xlabel('Wedge Depth')
    ylabel('Factor of Safety')
elseif strcmp(string,s4)==1
    figure(5) %Cohesion & FOS
    plot(CMatrix,FOSMatrix,'k.','Markersize',5)
    grid on
    xlabel('Cohesion (Pa)')
    ylabel('Factor of Safety')
elseif strcmp(string,s5)==1
    figure(6) %Friction Angle & FOS
    plot(PhMatrix,FOSMatrix,'k.','Markersize',5)
    grid on
    xlabel('Friction angle (Degrees)')
    ylabel('Factor of Safety')
elseif strcmp(string,s6)==1
    figure(7) %Density & FOS
    plot(PMatrix,FOSMatrix,'k.','Markersize',5)
    grid on
    xlabel('Density (kg/m3)')
    ylabel('Factor of Safety')
end






% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
grid on


% --- Executes on button press in plotplane.
function plotplane_Callback(hObject, eventdata, handles)
% hObject    handle to plotplane (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%RETREIVE INPUTS%%%%%%%%%%%%%%%%%%%%%%%%%%%%
H  = str2num(get(handles.Slopeht,'String'));   %Vertical height of slope (m)
H1 = str2num(get(handles.DaylHt,'String'));    %Vertical height to toe of discontinuity (m)
a  = str2num(get(handles.Slopeangle,'String'));%Slope angle, clockwise (a<90) (degrees)
t  = str2num(get(handles.JointAngle,'String'));%Discontinuity angle, clockwise (t<90 & t<a) (degrees)

vD    = str2num(get(handles.TensionDepth_u,'String'));%Mean Tension Crack Depth (m)

%Angle conversion
ar   = degtorad(a);
tr   = degtorad(t);
n0   = degtorad(90);
t0   = degtorad(180);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%DRAWING SLOPE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if vD > ((((H/cos(n0-ar)) - (H1/sin(ar)))*(sin(ar-tr)))/(sin(n0+tr)))

    %Tension Crack depth is not valid (too large)
    fprintf('Tension Crack Depth Not Valid\n\n')
    

else %Tension Crack depth is valid

    if ((((H/cos(n0-ar)) - (H1/sin(ar)))*((cos(tr))*(sin(ar-tr)))) - vD) >= 0
        %Is tension crack less than critical depth or at critical depth?

        if (((((H/cos(n0-ar)) - (H1/sin(ar)))*((cos(tr))*(sin(ar-tr)))) - vD)) == 0

            %Tension crack at critical depth
            c11  = (H/cos(n0-ar)) - (H1/sin(ar));
            x10 = (H1/sin(ar));
            x11 = c11;
            c12 = (sin(ar-tr))*(c11);
            x13 = (vD*tan(tr));
            x14 = (cos(ar-tr))*(c11);
            %fprintf('Critical Depth\n\n')
            Right=x11;
            Top=x13;
            Left=vD;
            Bottom=x14;
            A11 = (vD*x13)/2;
            A12 = (x12*x14)/2;
            A1  = A11+A12; %Area of Wedge

        else

            %Tension crack less than critical depth
            c01  = (H/cos(n0-ar)) - (H1/sin(ar));
            x00 = (H1/sin(ar));
            x01 = c01;
            x02 = (vD*tan(tr));
            x03 = (vD/cos(tr));
            x04 = ((sin(ar-tr))*c01) - (vD/cos(tr));
            x05 = ((x04)/(sin(tr)));
            x06 = ((x04)/(tan(tr)));
            x07 = ((cos(ar-tr))*c01);
            x08 = c01*sin(ar-tr);
            %fprintf('Less than Critical Depth\n\n')
            Right=x01;
            Top=x02 + x05;
            Left=vD;
            Bottom=x06 + x07;
            A01 = vD*x05;
            A02 = (x06*x04)/2;
            A03 = (x02*vD)/2;
            A04 = (x08*x07)/2;
            A0  = A01+A02+A03+A04; %Area of wedge

        end

    elseif ((((H/cos(n0-ar)) - (H1/sin(ar)))*((cos(tr))*(sin(ar-tr)))) - vD) <=0
        %Is tension crack greater than, equal or at a maximum value of critical depth?

        if (abs((((H/cos(n0-ar)) - (H1/sin(ar)))*((cos(tr))*(sin(ar-tr)))) - vD)) ==0

            %Tension crack at critical depth
            c11  = (H/cos(n0-ar)) - (H1/sin(ar));
            x10 = (H1/sin(ar));
            x11 = c11;
            c12 = (sin(ar-tr))*(c11);
            x13 = (vD*tan(tr));
            x14 = (cos(ar-tr))*(c11);
            %fprintf('Critical Depth\n\n')
            Right=x11;
            Top=x13;
            Left=vD;
            Bottom=x14;
            A11 = (vD*x13)/2;
            A12 = (x12*x14)/2;
            A1  = A11+A12; %Area of wedge

        else % Tension crack greater than critical depth or at edge of slope?

            if (((((H/cos(n0-ar)) - (H1/sin(ar)))*(sin(ar-tr)))/(sin(n0+tr)))-vD) ==0

                %Tension crack at edge of slope (maximum depth)
                c31 = (H/cos(n0-ar)) - (H1/sin(ar));
                x30 = (H1/sin(ar));
                x31 = c31;
                x32 = ((c31*sin(n0-ar))/(sin(n0+tr)));  
                %fprintf('Depth at Crest\n\n')
                Right = x31;
                Left = vD;
                Bottom = x32;
                Top = 0;
                A3 = ((((sin(n0-ar))*vD) * ((cos(n0-ar))*vD))/2) + ((((sin(ar-tr))*x32) * ((cos(ar-tr))*x32))/2);
                %Area of Wedge

             else

                %Tension crack greater than critical depth
                c21 = (H/cos(n0-ar)) - (H1/sin(ar));
                x20 = (H1/sin(ar));
                x21 = c21;
                x22 = (vD/sin(ar));
                x23 = c21 - (x22);
                x24 = (vD/tan(ar));
                x25 = (x23*(sin(ar-tr)))/(sin(tr));
                x26 = (x23*sin(t0-ar))/(sin(tr));
                x27 = (x25) - (x24);
                %fprintf('Greater than Critical Depth\n\n')
                Right= x21;
                Top= x27;
                Left=vD;
                Bottom=x26;
                A21 = (vD*x27);
                A22 = (vD*x24)/2;
                A231 = (((sin(tr))*(x25)*(cos(tr))*(x25))/2);
                A232 = (((sin(ar-tr))*(x23)*(cos(ar-tr))*(x23))/2);
                A23 = A231+ A232;
                A2  = A21 + A22 + A23; % Area of wedge

            end

        end

    end
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Hdraw  = H;
H1draw = H1;
Ddraw  = vD;
adraw  = degtorad(a);
tdraw  = degtorad(t);
Rdraw  = Right;
Tdraw  = Top;
Ldraw  = Left;
Bdraw  = Bottom;


%Plotting Slope Profile
hold on
plot([0,0+H1draw,(Tdraw+H1draw)],[Hdraw,Hdraw,Hdraw],'k','Linewidth',3)
plot([(Tdraw+H1draw),((Tdraw+ (Hdraw/(tan(adraw))))+H1draw)],[Hdraw,0],'k','Linewidth',3)
plot([((Tdraw+ (Hdraw/(tan(adraw)))-(H1draw/tan(adraw)))+H1draw),H1draw],[H1,(Hdraw-Ddraw)],'r--','Linewidth',2)
plot([H1draw,H1draw],[(Hdraw-Ddraw),Hdraw],'r--','Linewidth',2)
plot([((Tdraw+ (Hdraw/(tan(adraw))))+H1draw),(((Tdraw+ (Hdraw/(tan(adraw))))+H1draw)+2*H1draw)],[0,0],'k','Linewidth',3)
hold off
xlabel('X Distance (m)')
ylabel('Y Distance (m)')
axis([(0-H1draw),((((Tdraw+ (Hdraw/(tan(adraw))))+H1draw)+2*H1draw)+H1draw),(0-H1draw),(Hdraw+H1draw)])
grid on
guidata(hObject,handles)
