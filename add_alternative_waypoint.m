function wpt = add_alternative_waypoint(wpt,alt_wp,index)
    % alt_wp = [new_x, new_y, new_z]
    % index  = posisi wp ke-k yang ingin ditambahkan setelahnya (k+1)
    % new_r  = radius baru (opsional, default = 5)

    new_x = alt_wp(1);
    new_y = alt_wp(2);
    new_z = alt_wp(3);

    fprintf('\n Because new wp [%.1f %.1f %.1f] on index: %.0f \n',alt_wp(1),alt_wp(2),alt_wp(3),index)
    disp(wpt.pos.x)

    % Insert new waypoint at index+1
    wpt.pos.x   = [wpt.pos.x(1:index), new_x, wpt.pos.x(index+1:end)];
    wpt.pos.y   = [wpt.pos.y(1:index), new_y, wpt.pos.y(index+1:end)];
    wpt.pos.z   = [wpt.pos.z(1:index), new_z, wpt.pos.z(index+1:end)];


    disp('to')
    disp(wpt.pos.x)
