function virtual_tablet

    % Workspace in metri
    Xmin = 0.0; Xmax = 0.5;
    Ymin = 0.0; Ymax = 0.5;

    % Coordinate iniziali
    x = 0.0;
    y = 0.0;
    z = 0.5;

    % Salva nel workspace principale
    assignin('base','x',x);
    assignin('base','y',y);
    assignin('base','z',z);

    fig = figure;
    axis([Xmin Xmax Ymin Ymax]);
    grid on;
    hold on;

    set(fig, 'WindowButtonMotionFcn', @(src,event) mouseMove(src));
    set(fig, 'WindowButtonDownFcn', @(src,event) mouseClick(src, 'down'));
    set(fig, 'WindowButtonUpFcn', @(src,event) mouseClick(src, 'up'));

end

function mouseMove(~)
    C = get(gca, 'CurrentPoint');
    x = C(1,1);
    y = C(1,2);

    if x >= 0 && x <= 0.5 && y >= 0 && y <= 0.5
        % mantieni z corrente
        z = evalin('base','z');

        % Aggiorna nel workspace principale
        assignin('base','x',x);
        assignin('base','y',y);
        assignin('base','z',z);
    end
end

function mouseClick(~, mode)
    C = get(gca, 'CurrentPoint');
    x = C(1,1);
    y = C(1,2);

    switch mode
        case 'down'
            z = 0.1;
        case 'up'
            z = 0.5;
    end

    % Aggiorna nel workspace principale
    assignin('base','x',x);
    assignin('base','y',y);
    assignin('base','z',z);
end
