#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock VRC STTResult Publisher

VRC(음성 인식 컨트롤러)의 STT 결과를 발행하는 Mock 서버입니다.
음성 인식 결과를 시뮬레이션합니다.

Author: JAVIS Team
Date: 2025-10-27
"""

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from javis_dmc_test.nodes.mock_server_base import MockServerBase


class MockSTTResultPublisher(MockServerBase):
    '''Mock VRC STTResult Publisher
    
    음성 인식 결과를 주기적으로 발행합니다.
    
    mode='on': STT 결과 발행 (테스트용 문장)
    mode='off': 발행 중지
    '''
    
    def __init__(self):
        # 부모 클래스 초기화 (기본 mode='active')
        super().__init__('mock_vrc_stt_result')
        
        # STTResult Publisher 생성
        self._publisher = self.create_publisher(
            String,
            'dobby1/voice_recognition_controller/stt_result',
            10
        )
        
        # 테스트용 음성 명령 목록
        self._test_phrases = [
            '화장실 가고 싶어',
            '카페로 안내해줘',
            '출입구 어디야',
            '안내데스크로 가자',
            '취소'
        ]
        self._phrase_index = 0
        
        # 타이머 생성 (처음에는 중지)
        # 5초마다 발행 (실제 음성 인식보다 느림)
        self._timer = self.create_timer(5.0, self._timer_callback)
        self._timer.cancel()
        
        # mode 파라미터를 'off'로 변경
        self.set_parameters([Parameter('mode', Parameter.Type.STRING, 'off')])
        
        self.get_logger().info('STTResult Publisher 대기 (off 모드)')
        self.get_logger().info(f'테스트 문장: {self._test_phrases}')
    
    def on_mode_changed(self, old_mode: str, new_mode: str):
        '''모드 변경 시 호출되는 콜백 (부모 클래스 오버라이드)'''
        if new_mode == 'on':
            self._timer.reset()
            self._phrase_index = 0  # 처음부터 시작
            self.get_logger().info('STTResult Publisher 시작 (5초마다 발행)')
        else:
            self._timer.cancel()
            self.get_logger().info('STTResult Publisher 중지')
    
    def _timer_callback(self):
        '''타이머 콜백 - STTResult 발행'''
        # 테스트 문장 순환
        phrase = self._test_phrases[self._phrase_index]
        self._phrase_index = (self._phrase_index + 1) % len(self._test_phrases)
        
        # STT 결과 발행
        msg = String()
        msg.data = phrase
        
        self._publisher.publish(msg)
        
        self.get_logger().info(
            f'[Mock STT 결과] "{phrase}" '
            f'({self._phrase_index}/{len(self._test_phrases)})'
        )
        
        # v4.0 플로우 힌트
        if '취소' in phrase:
            self.get_logger().info('[v4.0 VRC 플로우] 취소 명령 → LISTENING 종료')
        else:
            self.get_logger().info(
                '[v4.0 VRC 플로우] '
                'STT 결과 → LLM Service → 좌표 획득 → RequestGuidance'
            )


def main(args=None):
    '''메인 함수'''
    rclpy.init(args=args)
    
    node = MockSTTResultPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
