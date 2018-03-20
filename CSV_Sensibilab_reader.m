%% Import data from text file.
% Script for importing data from CSV file saved by SENSIBILAB_IMU
% Written by PPerego 2018

%% Initialize variables.
[filename path] = uigetfile({'*.CSV';'*.csv'},'Seleziona file csv');
filename = [path filename];
delimiter = ',';

%% Read columns of data as text:
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%s%s%s%s%s%s%s%s%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'TextType', 'string',  'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Convert the contents of columns containing numeric text to numbers.
% Replace non-numeric text with NaN.
raw = repmat({''},length(dataArray{1}),length(dataArray)-1);
for col=1:length(dataArray)-1
    raw(1:length(dataArray{col}),col) = mat2cell(dataArray{col}, ones(length(dataArray{col}), 1));
end
numericData = NaN(size(dataArray{1},1),size(dataArray,2));

for col=[1,2,3,4,5,6,7,8,9]
    % Converts text in the input cell array to numbers. Replaced non-numeric
    % text with NaN.
    rawData = dataArray{col};
    for row=1:size(rawData, 1)
        % Create a regular expression to detect and remove non-numeric prefixes and
        % suffixes.
        regexstr = '(?<prefix>.*?)(?<numbers>([-]*(\d+[\,]*)+[\.]{0,1}\d*[eEdD]{0,1}[-+]*\d*[i]{0,1})|([-]*(\d+[\,]*)*[\.]{1,1}\d+[eEdD]{0,1}[-+]*\d*[i]{0,1}))(?<suffix>.*)';
        try
            result = regexp(rawData(row), regexstr, 'names');
            numbers = result.numbers;
            
            % Detected commas in non-thousand locations.
            invalidThousandsSeparator = false;
            if numbers.contains(',')
                thousandsRegExp = '^\d+?(\,\d{3})*\.{0,1}\d*$';
                if isempty(regexp(numbers, thousandsRegExp, 'once'))
                    numbers = NaN;
                    invalidThousandsSeparator = true;
                end
            end
            % Convert numeric text to numbers.
            if ~invalidThousandsSeparator
                numbers = textscan(char(strrep(numbers, ',', '')), '%f');
                numericData(row, col) = numbers{1};
                raw{row, col} = numbers{1};
            end
        catch
            raw{row, col} = rawData{row};
        end
    end
end


%% Replace non-numeric cells with NaN
R = cellfun(@(x) ~isnumeric(x) && ~islogical(x),raw); % Find non-numeric cells
raw(R) = {NaN}; % Replace non-numeric cells

%% Allocate imported array to column variable names
ACCX = cell2mat(raw(:, 1));
ACCY = cell2mat(raw(:, 2));
ACCZ = cell2mat(raw(:, 3));
GYROX = cell2mat(raw(:, 4));
GYROY = cell2mat(raw(:, 5));
GYROZ = cell2mat(raw(:, 6));
MAGX = cell2mat(raw(:, 7));
MAGY = cell2mat(raw(:, 8));
MAGZ = cell2mat(raw(:, 9));


%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans raw col numericData rawData row regexstr result numbers invalidThousandsSeparator thousandsRegExp R;

ACCX = ACCX+65535/2;
ACCX(ACCX>65535) = ACCX(ACCX>65535)-65535;
ACCY = ACCY+65535/2;
ACCY(ACCY>65535) = ACCY(ACCY>65535)-65535;
ACCZ = ACCZ+65535/2;
ACCZ(ACCZ>65535) = ACCZ(ACCZ>65535)-65535;
GYROX = GYROX+65535/2;
GYROX(GYROX>65535) = GYROX(GYROX>65535)-65535;
GYROY = GYROY+65535/2;
GYROY(GYROY>65535) = GYROY(GYROY>65535)-65535;
GYROZ = GYROZ+65535/2;
GYROZ(GYROZ>65535) = GYROZ(GYROZ>65535)-65535;
MAGX = MAGX+65535/2;
MAGX(MAGX>65535) = MAGX(MAGX>65535)-65535;
MAGY = MAGY+65535/2;
MAGY(MAGY>65535) = MAGY(MAGY>65535)-65535;
MAGZ = MAGZ+65535/2;
MAGZ(MAGZ>65535) = MAGZ(MAGZ>65535)-65535;

subplot(3,3,1)
plot(ACCX)
subplot(3,3,2)
plot(ACCY)
subplot(3,3,3)
plot(ACCZ)

subplot(3,3,4)
plot(GYROX)
subplot(3,3,5)
plot(GYROY)
subplot(3,3,6)
plot(GYROZ)

subplot(3,3,7)
plot(MAGX)
subplot(3,3,8)
plot(MAGY)
subplot(3,3,9)
plot(MAGZ)

