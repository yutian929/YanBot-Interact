from flask import Flask, request, jsonify

app = Flask(__name__)


@app.route("/query", methods=["POST"])
def handle_query():
    data = request.json
    query = data.get("query", "")
    print(f"Received query: {query}")
    return jsonify({"ans": "OK"})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8888)  # 必须绑定到 0.0.0.0
