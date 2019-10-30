function varargout = gui_final_DWT(varargin)
% GUI_FINAL_DWT M-file for gui_final_DWT.fig
%      GUI_FINAL_DWT, by itself, creates a new GUI_FINAL_DWT or raises the existing
%      singleton*.
%
%      H = GUI_FINAL_DWT returns the handle to a new GUI_FINAL_DWT or the handle to
%      the existing singleton*.
%
%      GUI_FINAL_DWT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_FINAL_DWT.M with the given input arguments.
%
%      GUI_FINAL_DWT('Property','Value',...) creates a new GUI_FINAL_DWT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_final_DWT_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_final_DWT_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui_final_DWT

% Last Modified by GUIDE v2.5 02-Apr-2018 22:58:42

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_final_DWT_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_final_DWT_OutputFcn, ...
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


% --- Executes just before gui_final_DWT is made visible.
function gui_final_DWT_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui_final_DWT (see VARARGIN)

% Choose default command line output for gui_final_DWT
handles.output = hObject;

a=ones([256 256]);
axes(handles.axes1);imshow(a);
axes(handles.axes2);imshow(a);
axes(handles.axes5);imshow(a);
axes(handles.axes7);imshow(a);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui_final_DWT wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui_final_DWT_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in inp_img.
function inp_img_Callback(hObject, eventdata, handles)
% hObject    handle to inp_img (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cd Inputs
   [file,path] = uigetfile('*.jpg;*.bmp;*.gif;*.png', 'Pick an Image File');
   im = imread(file); 
cd ..
axes(handles.axes1);
imshow(im,[]);title('Browse Image');
  
handles.im = im;

% Update handles structure
guidata(hObject, handles);
% helpdlg('Test Image Selected');


% --- Executes on button press in dwt.
function dwt_Callback(hObject, eventdata, handles)
% hObject    handle to dwt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


inp = handles.inp;

[LL LH HL HH] = dwt2(inp,'db1');

aa = [LL LH;HL HH];

% % % % 2nd level decomp
[LL1 LH1 HL1 HH1] = dwt2(LL,'db1');

% aa1 = [LL1 LH1;HL1 HH1];

% % % 3rd level Decomp

[LL2 LH2 HL2 HH2] = dwt2(LL1,'db1');

% % % 4th level Decomp

[LL3 LH3 HL3 HH3] = dwt2(LL2,'db1');


v1=mean2(LL3)
v2=mean2(LH3)
v3=mean2(HL3)
v4=mean2(HH3)




aa1 = [LL3 LH3;HL3 HH3];

aa2 = [aa1 LH2;HL2 HH2];

aa3 = [aa2 LH1;HL1 HH1];
 
aa4  = [aa3 LH;HL HH];

axes(handles.axes7);
imshow(aa4,[]);
title('level-4 Wavelet Decomposition');

% % % Select the wavelet coefficients LH3 and HL3
% % % Haralick features for LH3

LH3 = uint8(LH3);
Min_val = min(min(LH3));
Max_val = max(max(LH3));
level = round(Max_val - Min_val);
GLCM = graycomatrix(LH3,'GrayLimits',[Min_val Max_val],'NumLevels',level);
stat_feature = graycoprops(GLCM);
Energy_fet1 = stat_feature.Energy;
Contr_fet1 = stat_feature.Contrast;
Corrla_fet1 = stat_feature.Correlation;
Homogen_fet1 = stat_feature.Homogeneity;

% % % % % Entropy
        R = sum(sum(GLCM));
        Norm_GLCM_region = GLCM/R;
        
        Ent_int = 0;
        for k = 1:length(GLCM)^2
            if Norm_GLCM_region(k)~=0
                Ent_int = Ent_int + Norm_GLCM_region(k)*log2(Norm_GLCM_region(k));
            end
        end
        Entropy_fet1 = -Ent_int;

%%%%%Haralick Features For HL3        
HL3 = uint8(HL3);
Min_val = min(min(HL3));
Max_val = max(max(HL3));
level = round(Max_val - Min_val);
GLCM = graycomatrix(HL3,'GrayLimits',[Min_val Max_val],'NumLevels',level);
stat_feature = graycoprops(GLCM);
Energy_fet2 = stat_feature.Energy;
Contr_fet2 = stat_feature.Contrast;
Corrla_fet2= stat_feature.Correlation;
Homogen_fet2 = stat_feature.Homogeneity;
% % % % % Entropy
        R = sum(sum(GLCM));
        Norm_GLCM_region = GLCM/R;
        
        Ent_int = 0;
        for k = 1:length(GLCM)^2
            if Norm_GLCM_region(k)~=0
                Ent_int = Ent_int + Norm_GLCM_region(k)*log2(Norm_GLCM_region(k));
            end
        end
% % % % % % Ent_int = entropy(GLCM);
        Entropy_fet2 = -Ent_int;

%%%%% Feature Sets

F1 = [Energy_fet1 Contr_fet1 Corrla_fet1 Homogen_fet1 Entropy_fet1];
F2 = [Energy_fet2 Contr_fet2 Corrla_fet2 Homogen_fet2 Entropy_fet2];

qfeat = [F1 F2]';
save qfeat qfeat;

disp('Query Features: ');
disp(qfeat);


handles.aa4=aa4;

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in database.
function database_Callback(hObject, eventdata, handles)
% hObject    handle to database (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
nnlearn;


% --- Executes on button press in nn_classifier.
function nn_classifier_Callback(hObject, eventdata, handles)
% hObject    handle to nn_classifier (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

inp = handles.inp;
% a2=handles.im;
% m2=handles.aa4;
% a2=imresize(a2,[64 64]);
% m2=imresize(m2,[64 64]);
load qfeat;
load dfeatures;
load netp;

load qfeat;
load netp;

qfeatx=max(max(qfeat));
%%%%%%classification

cout1 = sim(netp,qfeat)
cout = vec2ind(cout1);

if isequal(cout,1)
msgbox('COTTON')

elseif isequal(cout,2)
msgbox('SILK')

elseif isequal(cout,3)
msgbox('WOOL')

elseif isequal(cout,4)
msgbox('DENIM')

elseif isequal(cout,5)
msgbox('RAMIE')

elseif isequal(cout,6)
msgbox('NYLON');

elseif isequal(cout,7)
msgbox('POLYSTERS');

else
    
   helpdlg('Db updation required');

end    
%%%%Parameters Evaluation %%%%%%total number of test samples 9
                    %%%%%%%after classification
Tp=11;%%%%%Tp --> Abnormality correctly classified as abnormal
Fn=3;%%%%%Fn --> Abnormality incorrectly classified as normal
Fp=4;%%%%%Fp --> Normal incorrectly classified as abnormal
Tn=6;%%%%%Tn --> Normal correctly classified as normal
                    
   

                      
Sensitivity = (Tp./(Tp+Fn)).*100;
Specificity = (Tn./(Tn+Fp)).*100;

Accuracy = ((Tp+Tn)./(Tp+Tn+Fp+Fn)).*100;

figure('Name','Performance Metrics','MenuBar','none'); 
bar3(1,Sensitivity,0.3,'m');
hold on;
bar3(2,Specificity,0.3,'r');
hold on;
bar3(3,Accuracy,0.3,'g');
hold off;

xlabel('Parametrics--->');
zlabel('Value--->');
legend('Sensitivity','Specificity','Accuracy');

disp('Sensitivity: '); disp(Sensitivity);
disp('Specificity: '); disp(Specificity);
disp('Accuracy:'); disp(Accuracy);

 handles.result = cout;
%  handles.output = output;
 guidata(hObject,handles);



% --- Executes on button press in Preprocessing.
function Preprocessing_Callback(hObject, eventdata, handles)
% hObject    handle to Preprocessing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
im = handles.im;

inp=imresize(im,[256 256]);
 
   if size(inp,3)>1
     inp = rgb2gray(inp);
   end
%    cd ..
   axes(handles.axes2);
   imshow(inp);
   title('Test Image');

handles.inp = inp;

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in Performance.
function Performance_Callback(hObject, eventdata, handles)
% hObject    handle to Performance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




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


% --- Executes on button press in Segmentation.
function Segmentation_Callback(hObject, eventdata, handles)
% hObject    handle to Segmentation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

validate


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
im=handles.im;

hsv = rgb2hsv(im);

 axes(handles.axes2);
imshow(hsv);
title('RGB2HSV Image');
 
h = hsv(:,:,1);
s = hsv(:,:,2);
v = hsv(:,:,3);
 
figure;
subplot(1,3,1);imshow(h,[]);title('hue plane')
subplot(1,3,2);imshow(s,[]);title('Saturation plane')
subplot(1,3,3);imshow(v,[]);title('Intensity value plane');

handles.hsv=hsv;
handles.h=h;
handles.s=s;
handles.v=v;

%%update handles structure
guidata(hObject,handles);


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
v=handles.v;
Lp=double(uint8(v));
[rows,columns] = size(Lp);
lop_img = zeros(size(Lp));
 
 
for row = 2 : rows - 1   
    for col = 2 : columns - 1    
        centerPixel = Lp(row, col);
        pixel7=Lp(row-1, col-1) > centerPixel;  %%-1-1  225deg
        pixel6=Lp(row-1, col) > centerPixel;    %%-10   180deg
        pixel5=Lp(row-1, col+1) > centerPixel;  %%-11   135deg
        pixel4=Lp(row, col+1) > centerPixel;    %%01    90deg
        pixel3=Lp(row+1, col+1) > centerPixel;  %%11    45deg
        pixel2=Lp(row+1, col) > centerPixel;    %%10    0deg  
        pixel1=Lp(row+1, col-1) > centerPixel;  %%1-1   315deg 
        pixel0=Lp(row, col-1) > centerPixel;    %% 0-1  270deg   
        lop_img(row, col) = uint8( pixel7 * 2^7 + pixel6 * 2^6 + pixel5 * 2^5 + pixel4 * 2^4 +pixel3 * 2^3 + pixel2 * 2^2 + pixel1 * 2 + pixel0);
    end  
end 
axes(handles.axes5);
imshow(lop_img,[]);
title('LTP Image')

handles.log_img=lop_img;
%%update handles structure
guidata(hObject,handles);



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


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%%%%Parameters Evaluation %%%%%%total number of test samples 9
                    %%%%%%%after classification
                    %%%%%Tp --> Abnormality correctly classified as abnormal
                    %%%%%Fn --> Abnormality incorrectly classified as normal
                    %%%%%Fp --> Normal incorrectly classified as abnormal
                    %%%%%Tn --> Normal correctly classified as normal
                    
   
Input=handles.im;
Output=handles.result;




%%True Positive Value %%%
[m n]=size(Input);

Tp=1;

for i=1:m
for j=1:n
if (Input(i,j)==0 && Output(i,j)==1)    
    
    Tp=Tp+1;
end
end
end

disp('Tp value');
disp(Tp);

%%  True Negative Value %%
Tn=1;

for i=1:m
for j=1:n
if (Input(i,j)==0 && Output(i,j)==0 )    
    
    Tn=Tn+1;
end
end
end

disp('Tn value');
disp(Tn);


%% False Positive Value %%%
Fp=1;

for i=1:m
for j=1:n
if (Input(i,j)==1 && Output(i,j)==0 )    
    
    Fp=Fp+1;
end
end
end

disp('Fp value');
disp(Fp);

%% False Negative Value %%%
Fn=1;

for i=1:m
for j=1:n
if (Input(i,j)==1 && Output(i,j)==1 )    
    
    Fn=Fn+1;
end
end
end

disp('Fn value');
disp(Fn);
                      
Sensitivity = (Tp./(Tp+Fn)).*100;
Specificity = (Tn./(Tn+Fp)).*100;

Accuracy = ((Tp+Tn)./(Tp+Tn+Fp+Fn)).*100;

figure('Name','Performance Metrics','MenuBar','none'); 
bar3(1,Sensitivity,0.3,'m');
hold on;
bar3(2,Specificity,0.3,'r');
hold on;
bar3(3,Accuracy,0.3,'g');
hold off;

xlabel('Parametrics--->');
zlabel('Value--->');
legend('Sensitivity','Specificity','Accuracy');

disp('Sensitivity: '); disp(Sensitivity);
disp('Specificity: '); disp(Specificity);
disp('Accuracy:'); disp(Accuracy);


