function response = cm_send(cm, command)
%CM_SEND CarMaker ScriptControl 명령 전송 및 응답 수신
%
%   response = CM_SEND(cm, command)
%
%   Inputs:
%       cm      - (tcpclient) TCP 클라이언트 객체
%       command - (char) ScriptControl 명령 문자열
%
%   Outputs:
%       response - (char) CarMaker 응답 문자열

    % 명령 전송 (줄바꿈 포함)
    write(cm, uint8([command char(10)]));

    % 응답 대기
    pause(0.2);
    maxWait = 5.0;  % 최대 대기 [s]
    elapsed = 0.2;

    while cm.BytesAvailable == 0 && elapsed < maxWait
        pause(0.1);
        elapsed = elapsed + 0.1;
    end

    % 응답 읽기
    if cm.BytesAvailable > 0
        raw = read(cm);
        response = strtrim(char(raw));
    else
        response = '';
        warning('[cm_send] No response for command: %s', command);
    end

end
