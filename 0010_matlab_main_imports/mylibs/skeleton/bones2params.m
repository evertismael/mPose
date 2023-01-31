function [p0, abc, rot_vect2glb, names, pr] = bones2params(bones, Nfrms)
    p0 = zeros(3,size(bones,2), Nfrms);
    pr = zeros(1,size(bones,2));
    abc = zeros(3,size(bones,2), Nfrms);
    rot_vect2glb = zeros(4,size(bones,2), Nfrms);
    names = repmat({},1,size(bones,2));
    for bone_idx = 1:size(bones,2)
        % name:
        names{bone_idx} = bones{bone_idx}.name;
        % bone center:
        p0(:,bone_idx,:) = bones{bone_idx}.p0;
        % reflective probabilities:
        pr(:,bone_idx) = bones{bone_idx}.pr;

        % bone mayor and minor axes:
        abc(1,bone_idx,:) = bones{bone_idx}.a;
        abc(2,bone_idx,:) = bones{bone_idx}.b;
        abc(3,bone_idx,:) = bones{bone_idx}.c;
        
        % rotation angles:
        for frm_idx = 1:Nfrms
            rot_vect2glb(:,bone_idx,frm_idx) = vrrotvec([0 0 1].',bones{bone_idx}.uz(:,1,frm_idx));
        end
    end    
end

