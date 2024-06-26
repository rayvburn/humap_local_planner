function pose = subtractPoses(pose_ref, pose_other)
    pose = [...
        pose_ref(1) - pose_other(1); ...
        pose_ref(2) - pose_other(2); ...
        pose_ref(3) - pose_other(3)  ...
    ];
end
