function info = student_info()
%STUDENT_INFO 학생 정보 — 채점 시 매칭에 사용. **반드시 수정해서 제출.**
%
%   info struct fields:
%       .student_id   - 학번 (string, 예: '20210123')
%       .name         - 이름 (string)
%       .team_members - 팀 구성원 (cell array of struct, 개인 제출 시 빈 cell)
%       .course       - 과목 코드/이름
%       .ai_usage     - AI 도구 (ChatGPT, Claude 등) 사용 여부 + 범위 (string)
%
%   본 파일을 수정하지 않으면 -5점 감점 + 채점 시트 매칭 불가.

    info.student_id   = '202624331';
    info.name         = '김현서';

    info.team_members = {};   % 2인 팀이면 {struct('id','...','name','...')} 추가

    info.course = '자동제어 - 2026 봄';

    % AI 도구 사용 사실 (정직 신고) — 과제 규정(ASSIGNMENT §7)상 필수
    info.ai_usage = ['Claude (Anthropic) used for controller architecture design, ', ...
                     'gain tuning iteration, and report drafting. ', ...
                     'All results verified by re-running grade.m locally.'];

    %% 검증 (수정 금지)
    if contains(info.student_id, 'TODO_FILL')
        warning('[student_info] 학번이 기입되지 않았습니다 — 채점 시 감점 + 매칭 불가');
    end
    if contains(info.name, 'TODO_FILL')
        warning('[student_info] 이름이 기입되지 않았습니다');
    end
end
