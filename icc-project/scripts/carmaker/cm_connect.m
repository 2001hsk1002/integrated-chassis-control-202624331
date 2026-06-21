function cm = cm_connect(host, port)
%CM_CONNECT CarMaker ScriptControl TCP 연결
%
%   cm = CM_CONNECT()
%   cm = CM_CONNECT(host, port)
%
%   CarMaker의 ScriptControl TCP 인터페이스에 연결하고 상태를 확인한다.
%
%   Inputs:
%       host - (char, optional) 호스트 주소 (default: 'localhost')
%       port - (double, optional) TCP 포트 (default: 16660)
%
%   Outputs:
%       cm - (tcpclient) TCP 클라이언트 객체

    if nargin < 1, host = 'localhost'; end
    if nargin < 2, port = 16660; end

    fprintf('[cm_connect] Connecting to CarMaker at %s:%d ... ', host, port);

    try
        cm = tcpclient(host, port, 'Timeout', 10);
        pause(0.5);

        % 상태 확인
        response = cm_send(cm, 'GetStatus');
        fprintf('OK\n');
        fprintf('[cm_connect] CarMaker Status: %s\n', response);
    catch ME
        error('[cm_connect] Connection failed: %s\nIs CarMaker running with ScriptControl enabled?', ...
              ME.message);
    end

end
