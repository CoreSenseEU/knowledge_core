#!/usr/bin/env python3

"""Simple terminal viewer for the KnowledgeCore knowledge base.

Clears the screen and prints all facts in the KB, refreshing periodically.
"""

import argparse
import json
import os
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from kb_msgs.srv import Query
except ImportError:
    print("Unable to load ROS 2 support. Make sure to source your ROS 2 workspace.")
    sys.exit(1)


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def fetch_all_facts(node, query_client):
    req = Query.Request()
    req.patterns = ["?s ?p ?o"]
    req.vars = []
    req.models = []

    future = query_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    result = future.result()
    if result is None:
        return None, "Service call timed out"
    if not result.success:
        return None, result.error_msg

    triples = json.loads(result.json)
    return triples, None


def print_facts(triples, kb_namespace):
    print(f"KnowledgeCore KB viewer — namespace: {kb_namespace}")
    print(f"Facts: {len(triples)}")
    print("=" * 60)
    if not triples:
        print("(knowledge base is empty)")
    else:
        for triple in sorted(triples, key=lambda t: (t.get("s", ""), t.get("p", ""), t.get("o", ""))):
            s = triple.get("s", "?")
            p = triple.get("p", "?")
            o = triple.get("o", "?")
            print(f"  {s}  {p}  {o}")
    print("=" * 60)


def main():
    argv = sys.argv[1:sys.argv.index("--ros-args")] \
        if "--ros-args" in sys.argv else sys.argv[1:]

    parser = argparse.ArgumentParser(description="KnowledgeCore terminal viewer")
    parser.add_argument(
        "--ns", "--namespace",
        default="/kb",
        dest="namespace",
        help="KB namespace (default: /kb)"
    )
    parser.add_argument(
        "-r", "--rate",
        default=1.0,
        type=float,
        help="Refresh rate in Hz (default: 1.0)"
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node("kb_text_viewer")

    query_service = args.namespace.rstrip("/") + "/query"
    query_client = node.create_client(Query, query_service)

    print(f"Waiting for service {query_service}...")
    if not query_client.wait_for_service(timeout_sec=10.0):
        print(f"ERROR: Service {query_service} not available.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    period = 1.0 / args.rate

    try:
        while rclpy.ok():
            triples, error = fetch_all_facts(node, query_client)
            clear_screen()
            if error:
                print(f"ERROR: {error}")
            else:
                print_facts(triples, args.namespace)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
