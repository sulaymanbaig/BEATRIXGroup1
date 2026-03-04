function beatrix_face_official_kinematics()
    % BEATRIX: Official Kinematic Face Tracking
    % Uses exact X/Z differential mapping from the project brief.
    
    %% ----- SETTINGS -----
    useRobot = true; 
    camIdx = 2; 
    
    deadbandX = 90;    
    speed = 220;       
    sendPeriod = 0.15; 
    
    % *** OFFICIAL SAFETY LIMIT ***
    maxPan = 300; % The document explicitly states 300 is the max safe limit for X/Z
    
    % Initialize
    try
        delete(findobj('Type', 'udpport')); 
        if exist('cam','var'), clear cam; end
        cam = webcam(camIdx);
        cam.Resolution = '640x480';
        detector = mtcnn.Detector("MinSize", 40);
        disp('Face Tracking: Official Kinematics Active.');
    catch
        error("Camera error.");
    end
    
    % Serial Setup
    s = []; motorsEnabled = false;
    if useRobot
        try
            s = serialport("COM3", 9600); 
            configureTerminator(s, "CR");
            pause(2); flush(s);
            writeline(s, "@ENMOTORS ON"); 
            pause(0.1);
            writeline(s, "@CALNOW");
            motorsEnabled = true;
        catch
            disp("Check COM port!");
        end
    end
    
    % The Master Horizontal Accumulator
    currentPan = 0; 
    tLastSend = tic;
    
    fig = figure('Name','BEATRIX: Official Tracking', 'NumberTitle', 'off');
    
    %% ----- MAIN LOOP -----
    while ishandle(fig)
        frame = flip(snapshot(cam), 2);
        [bboxes, scores, ~] = detector.detect(frame);
        
        if ~isempty(bboxes)
            [~, i] = max(scores); bb = bboxes(i,:);
            cx = bb(1) + bb(3)/2;
            frame = insertShape(frame, "Rectangle", bb, "Color", "cyan", "LineWidth", 4);
            
            errX = cx - 320; 
            
            % --- TRACKING LOGIC ---
            if abs(errX) > deadbandX 
                
                if toc(tLastSend) > sendPeriod
                    
                    % Proportional step size (Anti-Stiction tuned)
                    dynamicStep = round(abs(errX) * 0.1); 
                    dynamicStep = max(10, min(dynamicStep, 30)); 
                    
                    dir = sign(errX); 
                    
                    % Update the master position, clamped to the official 300 limit
                    currentPan = currentPan + (dir * dynamicStep);
                    currentPan = max(-maxPan, min(maxPan, currentPan));
                    
                    % ---------------------------------------------------------
                    % THE OFFICIAL BEATRIX DIFFERENTIAL MAPPING
                    % ---------------------------------------------------------
                    % Text says: x and z move in opposite directions for panning.
                    % Assuming Motor 1 = x, Motor 2 = y, Motor 3 = z
                    
                    motX = currentPan;       % X moves normal
                    motY = 0;                % Y stays locked at zero for horizontal pan
                    motZ = -currentPan;      % Z moves exactly opposite to X
                    
                    % Send absolute coordinate using the mapped variables
                    cmdStr = sprintf("@MOVAALL %d %d %d %d %d %d", ...
                                     motX, motY, motZ, speed, speed, speed);
                    
                    if useRobot && motorsEnabled
                        writeline(s, cmdStr);
                        tLastSend = tic;
                    end
                end
            end
        end
        
        imshow(frame); drawnow limitrate;
    end
    
    if useRobot && ~isempty(s), clear s; end
    if exist('cam','var'), clear cam; end
end