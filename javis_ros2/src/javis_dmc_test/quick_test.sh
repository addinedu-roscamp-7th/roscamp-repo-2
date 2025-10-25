#!/bin/bash

# javis_dmc_test 빠른 테스트 스크립트
# 사용법: ./quick_test.sh

set -e

WORKSPACE_DIR=~/dev_ws/roscamp-repo-2/javis_ros2

echo "=================================="
echo "  JAVIS DMC Test - Quick Test"
echo "=================================="
echo ""

# 색상 정의
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Step 1: 빌드 확인
echo -e "${YELLOW}[Step 1/5] 빌드 확인...${NC}"
cd $WORKSPACE_DIR

if [ ! -d "install/javis_dmc_test" ]; then
    echo -e "${RED}❌ javis_dmc_test 패키지가 빌드되지 않았습니다.${NC}"
    echo "다음 명령어로 빌드하세요:"
    echo "  cd $WORKSPACE_DIR"
    echo "  colcon build --packages-select javis_dmc_test_msgs javis_dmc_test --symlink-install"
    exit 1
fi

echo -e "${GREEN}✅ 패키지 빌드 확인 완료${NC}"
echo ""

# Step 2: 환경 변수 로드
echo -e "${YELLOW}[Step 2/5] 환경 변수 로드...${NC}"
source $WORKSPACE_DIR/install/setup.bash
echo -e "${GREEN}✅ 환경 변수 로드 완료${NC}"
echo ""

# Step 3: 패키지 확인
echo -e "${YELLOW}[Step 3/5] 패키지 확인...${NC}"

if ros2 pkg list | grep -q "javis_dmc_test"; then
    echo -e "${GREEN}✅ javis_dmc_test 패키지 발견${NC}"
else
    echo -e "${RED}❌ javis_dmc_test 패키지를 찾을 수 없습니다.${NC}"
    exit 1
fi

if ros2 pkg list | grep -q "javis_dmc_test_msgs"; then
    echo -e "${GREEN}✅ javis_dmc_test_msgs 패키지 발견${NC}"
else
    echo -e "${RED}❌ javis_dmc_test_msgs 패키지를 찾을 수 없습니다.${NC}"
    exit 1
fi
echo ""

# Step 4: 실행 파일 확인
echo -e "${YELLOW}[Step 4/5] 실행 파일 확인...${NC}"

EXECUTABLES=$(ros2 pkg executables javis_dmc_test)
if echo "$EXECUTABLES" | grep -q "mock_bridge_node"; then
    echo -e "${GREEN}✅ mock_bridge_node 발견${NC}"
else
    echo -e "${RED}❌ mock_bridge_node를 찾을 수 없습니다.${NC}"
    exit 1
fi

if echo "$EXECUTABLES" | grep -q "mock_rcs_node"; then
    echo -e "${GREEN}✅ mock_rcs_node 발견${NC}"
else
    echo -e "${RED}❌ mock_rcs_node를 찾을 수 없습니다.${NC}"
    exit 1
fi
echo ""

# Step 5: 서비스 타입 확인
echo -e "${YELLOW}[Step 5/5] 서비스 타입 확인...${NC}"

if ros2 interface list | grep -q "javis_dmc_test_msgs/srv/SetMockMethod"; then
    echo -e "${GREEN}✅ SetMockMethod 서비스 발견${NC}"
else
    echo -e "${RED}❌ SetMockMethod 서비스를 찾을 수 없습니다.${NC}"
    exit 1
fi

if ros2 interface list | grep -q "javis_dmc_test_msgs/srv/SendTask"; then
    echo -e "${GREEN}✅ SendTask 서비스 발견${NC}"
else
    echo -e "${RED}❌ SendTask 서비스를 찾을 수 없습니다.${NC}"
    exit 1
fi
echo ""

echo "=================================="
echo -e "${GREEN}✅ 모든 테스트 통과!${NC}"
echo "=================================="
echo ""
echo "다음 명령어로 노드를 실행할 수 있습니다:"
echo ""
echo "  # Launch 파일로 전체 실행 (권장)"
echo "  ros2 launch javis_dmc_test test_full.launch.py"
echo ""
echo "  # 개별 노드 실행"
echo "  ros2 run javis_dmc_test mock_bridge_node"
echo "  ros2 run javis_dmc_test mock_rcs_node"
echo ""
echo "자세한 테스트 절차는 TESTING.md를 참조하세요."
echo ""
