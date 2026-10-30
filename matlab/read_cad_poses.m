function pose = read_cad_poses(xml_file)
% READ_CAD_POSES  Where every part instance sits in the SolidWorks assembly.
%   pose = read_cad_poses('gryphon.xml')
%   pose.<instance>.R : rotation of the part frame (3x3)
%   pose.<instance>.t : position of the part frame in mm
% The instance names are made valid field names, e.g. 'part8-shaft3-1' -> part8_shaft3_1
doc = xmlread(xml_file);
list = doc.getElementsByTagName('Instance');
pose = struct();
for i = 0:list.getLength - 1
    node = list.item(i);
    uid = char(node.getAttribute('uid'));
    R = sscanf(char(node.getElementsByTagName('Rotation').item(0).getTextContent), '%f');
    t = sscanf(char(node.getElementsByTagName('Translation').item(0).getTextContent), '%f');
    pose.(matlab.lang.makeValidName(uid)).R = reshape(R, 3, 3)';   % the file lists the rows
    pose.(matlab.lang.makeValidName(uid)).t = 1000 * t(:);         % m -> mm
end
end
