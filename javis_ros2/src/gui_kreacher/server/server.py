# test_server.py
from flask import Flask, request, jsonify

app = Flask(__name__)

# OrderID 카운터 (실제로는 DB 사용)
order_counter = 0

@app.route('/', methods=['POST'])
def create_order():
    global order_counter
    
    data = request.json
    print("받은 주문:", data)
    
    # OrderID 자동 증가
    order_counter += 1
    
    # 응답 반환
    response = {
        "OrderID": order_counter,
        "OrderStatus": "접수"
    }
    
    print(f"생성된 OrderID: {order_counter}")
    return jsonify(response)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=True)
