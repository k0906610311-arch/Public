import os
from flask import Flask, jsonify, request

app = Flask(__name__)

# 原本的首頁測試路由
@app.route("/")
def index():
    return jsonify({"status": "success", "message": "API running on Render!"})

# 新增：接收 Arduino 分藥機連線請求的路由
@app.route("/getdata.php", methods=["GET"])
def get_dispenser_schedule():
    uid = request.args.get("uid")
    print(f">>> 收到分藥機連線，機器 UID: {uid}")
    
    # 測試排程：前 9 碼為藥管顆數 (第 1 管出 1 顆)，後 4 碼為時間 (例如 1200 代表 12:00)
    # 若目前無排程可改成 return "NODATA"
    test_schedule = "1000000001200"
    return test_schedule

if __name__ == "__main__":
    port = int(os.environ.get("PORT", 10000))
    app.run(host="0.0.0.0", port=port)