function success = cm_wait_idle(cm, timeout)
%CM_WAIT_IDLE CarMaker 시뮬레이션 완료 대기
%
%   success = CM_WAIT_IDLE(cm, timeout)
%
%   CarMaker 상태가 'Idle'이 될 때까지 폴링한다.
%
%   Inputs:
%       cm      - (tcpclient) TCP 클라이언트
%       timeout - (double) 최대 대기 시간 [s] (default: 120)
%
%   Outputs:
%       success - (logical) true = Idle 도달, false = 타임아웃

    if nargin < 2, timeout = 120; end

    POLL_INTERVAL = 0.5;  % [s]
    t0 = tic;
    success = false;

    while toc(t0) < timeout
        status = cm_send(cm, 'GetStatus');

        if contains(status, 'Idle', 'IgnoreCase', true)
            success = true;
            fprintf('[cm_wait_idle] Simulation completed (%.1f s)\n', toc(t0));
            return;
        end

        if contains(status, 'Error', 'IgnoreCase', true)
            warning('[cm_wait_idle] CarMaker error detected: %s', status);
            return;
        end

        pause(POLL_INTERVAL);
    end

    warning('[cm_wait_idle] Timeout after %.0f s. Status: %s', timeout, status);

end
