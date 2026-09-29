import sys

try:
    import rclpy
    from rclpy.node import Node
    import ament_index_python as aip
except ImportError:
    print(
        "Unable to load the ROS 2 support. You might want to run with --no-ros"
        " to use KnowledgeCore without ROS 2 support enabled (or check an older"
        " version for ROS 1 support)."
    )
    sys.exit(1)

import collections
import json

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from knowledge_core.exceptions import KbServerError

from kb_msgs.srv import Manage, Revise, Query, About, Lookup, Sparql, Event
from kb_msgs.msg import ActiveConcepts
from std_msgs.msg import String
from as2_msgs.srv import RegisterModule
from as2_msgs.msg import LocalGenericMessage
from rclpy.serialization import deserialize_message, serialize_message

EVENTS_TOPIC_NS = "events/"

DIAGNOSTICS_FREQUENCY = 1  # Hz


def to_sec(duration_msg):
    return duration_msg.sec + duration_msg.nanosec / 1e9


class KnowledgeCoreROS(Node):
    """
    ROS wrapper around the KnowledgeCore knowledge base.

    To try it:
    ```
    import sys
    import rclpy

    from knowledge_core.kb import KnowledgeCore
    from knowledge_core.knowledge_core_ros import KnowledgeCoreROS

    kb = KnowledgeCore()

    rclpy.init(args=sys.argv)

    ros = KnowledgeCoreROS(kb)

    rclpy.spin()
    ```

    You can then interact with the knowledge base via its topics and services.
    """

    def __init__(self, kb):
        super().__init__('knowledge_core')

        self.kb = kb

        self.declare_parameter('default_kb', rclpy.Parameter.Type.STRING)
        default_kb = self.get_parameter_or('default_kb', None)

        RES_NAME = "ontology"

        if default_kb.value:
            self.get_logger().info(
                f"Default knowledge base file: {default_kb.value}")
            if not default_kb.value.startswith(f"{RES_NAME}://"):
                self.get_logger().error(
                    "Invalid default knowledge base URI."
                    f" Should be in the form '{RES_NAME}://<package>/<file>'")
                rclpy.shutdown()
                return

            pkg, res = default_kb.value.split("://")[1].split("/")

            try:
                ontologies_res = aip.get_resource(RES_NAME, pkg)
            except LookupError:
                self.get_logger().error(
                    f"Default ontology (default_kb) set to {default_kb.value}, "
                    f"yet package '{pkg}' not found in the '{RES_NAME}' resource index")
                rclpy.shutdown()
                return

            ontologies_list = {
                r.split("/")[1]: r for r in ontologies_res[0].split("\n")}

            if res not in ontologies_list.keys():
                self.get_logger().error(
                    f"Default ontology (default_kb) set to {default_kb.value}, "
                    "yet resource '{res}' not found in the '{RES_NAME}' resource "
                    "index (pkg '{pkg}'). Found files: {list(ontologies_list.keys())}")
                rclpy.shutdown()
                return

            path = aip.packages.get_package_share_path(
                pkg) / ontologies_list[res]

            self.get_logger().info(
                f"Loading default knowledge base [{default_kb.value}] from {path}")
            self.kb.add_default_ontology(path)
        else:
            self.get_logger().info(
                "No default knowledge base file provided. Starting with an empty one.")

        self.last_facts = collections.deque(maxlen=5)

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 1
        )
        self.last_diagnostics_ts = self.get_clock().now()

        self.update_sub = self.create_subscription(
            String, "add_fact", self.on_update_fact, 200)
        self.retract_sub = self.create_subscription(
            String, "remove_fact", self.on_retract_fact, 200)

        self.active_concepts_pub = self.create_publisher(
            ActiveConcepts, "active_concepts", 1)
        self._current_active_concepts = set()

        self.create_service(Manage, "manage",  self.handle_manage)
        self.create_service(Revise, "revise",  self.handle_revise)
        self.create_service(Query, "query",  self.handle_query)
        self.create_service(About, "about",  self.handle_about)
        self.create_service(About, "label",  self.handle_details)
        self.create_service(About, "details",  self.handle_details)
        self.create_service(Lookup, "lookup",  self.handle_lookup)
        self.create_service(Event, "events",  self.handle_new_event)
        self.create_service(Sparql, "sparql",  self.handle_sparql)

        # Register with ca_gateway. The gateway lives one namespace level above
        # (e.g. /drone0 when this node is at /drone0/kb).
        ns = self.get_namespace()  # e.g. "/drone0/kb" or "/kb"
        parent_ns = ns.rsplit('/', 1)[0]  # "/drone0" or ""
        register_service = parent_ns + '/register_module'
        self._gateway_out_topic = parent_ns + '/gateway_out'
        self._register_client = self.create_client(RegisterModule, register_service)
        self._gateway_out_pub = self.create_publisher(
            LocalGenericMessage, self._gateway_out_topic, 10)
        self._local_query_client = self.create_client(Query, 'query')
        # Retry every second until the gateway is available
        self._register_timer = self.create_timer(1.0, self._register_with_gateway)

        self.get_logger().info(
            """
KnowledgeCore
=============

Knowledge base started.

Available topics:
- /kb/add_fact [std_msgs/String]
- /kb/remove_fact [std_msgs/String]
- /kb/active_concepts [kb_msgs/ActiveConcepts]
- /kb/events/<id> [std_msgs/String] for
  each subscribed event

Available services:
- /kb/manage [kb_msgs/Manage]
- /kb/revise [kb_msgs/Revise]
- /kb/query [kb_msgs/Query]
- /kb/about [kb_msgs/About]
- /kb/label [kb_msgs/About]
- /kb/details [kb_msgs/About]
- /kb/lookup [kb_msgs/Lookup]
- /kb/sparql [kb_msgs/Sparql]
- /kb/events [kb_msgs/Event]

"""
        )

    def on_update_fact(self, msg):
        # A single message may carry several newline-separated triples
        # (see KBInterface::add_facts) so that a bulk write only triggers
        # one reasoner materialisation pass instead of one per triple.
        stmts = msg.data.split("\n")
        try:
            self.kb.update(stmts)
        except KbServerError as kse:
            self.get_logger().error(str(kse))
        for stmt in stmts:
            if stmt not in self.last_facts:
                self.last_facts.append(stmt)

    def on_retract_fact(self, msg):
        try:
            self.kb.remove([msg.data])
        except KbServerError as kse:
            self.get_logger().error(str(kse))
        if msg.data in self.last_facts:
            self.last_facts.remove(msg.data)

    def handle_manage(self, req, response):

        from kb_msgs.srv import Manage

        try:
            if req.action == Manage.Request.CLEAR:
                if "keep_defaults" in req.parameters:
                    self.kb.clear(keep_defaults=True)
                else:
                    self.kb.clear()
                response.success = True
                response.error_msg = ""
                return response

            elif req.action == Manage.Request.LOAD:
                if len(req.parameters) != 1:
                    response.success = False
                    response.error_msg = "'load' expects 'parameters' to contain exactly " \
                                         "one URI pointing to the ontology to load"
                    return response

                self.kb.load(req.parameters[0], models=req.models)
                response.success = True
                response.error_msg = ""
                return response

            elif req.action == Manage.Request.SAVE:
                if len(req.parameters) != 2:
                    response.success = False
                    response.error_msg = "'save' expects 'parameters' to contain "
                    "exactly two values: the path and the basename. The "
                    "knowledge base will be saved as "
                    "`path/basename-<model>.xml (one file per model)"
                    return response

                self.kb.save(
                    req.parameters[0], req.parameters[1], models=req.models)
                response.success = True
                response.error_msg = ""
                return response

            elif req.action == Manage.Request.STATUS:
                status = {
                    "name": self.kb.hello(),
                    "version": self.kb.version(),
                    "reasoning_enabled": self.kb.reasoner_enabled,
                }
                response.success = True
                response.json = json.dumps(status)
                response.error_msg = ""
                return response
            else:
                response.success = False
                response.error_msg = "Unknown 'manage' action: %s" % req.action
                return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_revise(self, req, response):

        policy = {
            "method": req.method,
            "models": req.models,
            "lifespan": to_sec(req.lifespan),
        }
        try:
            self.kb.revise(req.statements, policy)

            response.success = True
            response.error_msg = ""
            return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_query(self, req, response):

        try:
            res = self.kb.find(req.patterns, req.vars, req.models)
            response.success = True
            response.json = json.dumps(res)
            response.error_msg = ""
            return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_about(self, req, response):

        try:
            res = self.kb.about(req.term, req.models)
            response.success = True
            response.json = json.dumps(res)
            response.error_msg = ""
            return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_label(self, req, response):

        try:
            res = self.kb.label(req.term, req.models)
            response.success = True
            response.json = json.dumps(res)
            response.error_msg = ""
            return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_details(self, req, response):

        try:
            res = self.kb.details(req.term, req.models)
            response.success = True
            response.json = json.dumps(res)
            response.error_msg = ""
            return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_lookup(self, req, response):

        try:
            res = self.kb.lookup(req.query, req.models)
            response.success = True
            response.json = json.dumps(res)
            response.error_msg = ""
            return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_sparql(self, req, response):

        try:
            res = self.kb.sparql(req.query, req.models)
            response.success = True
            response.json = json.dumps(res)
            response.error_msg = ""
            return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_new_event(self, req, response):

        class EvtRelay:
            """
            Inner class to 'pretend' the ROS backend is a regular 'client' to the KB server.

            This is required for events as they are asynchronous -> the
            knowledge base server needs to keep a handle to all the 'clients'
            that have subscribed to an event for later notification.

            See eg KnowledgeCore.onupdate()
            """

            def __init__(self, node, evt_id, kb):
                self.evt = evt_id
                self.node = node
                self.kb = kb
                self.has_been_subscribed = False
                self.pub = self.node.create_publisher(
                    String, EVENTS_TOPIC_NS + evt_id, 1)

            def __eq__(self, other):
                return self.evt == other.evt

            def __hash__(self):
                return hash(self.evt)

            def sendmsg(self, msg):

                if self.pub.get_subscription_count() == 0:
                    if self.has_been_subscribed:
                        self.node.get_logger().warn(
                            "No one listing to event <%s> anymore; removing it."
                            % self.evt
                        )
                        self.node.destroy_publisher(self.pub)
                        self.kb.eventsubscriptions[self.evt].remove(self)
                else:
                    self.has_been_subscribed = True

                    evt = msg[1]
                    self.node.get_logger().info(
                        "Event <" + evt.id + "> triggered. Notifying ROS clients."
                    )
                    evt_msg = String()
                    evt_msg.data = json.dumps(evt.content)
                    self.pub.publish(evt_msg)

        try:
            evt = self.kb.subscribe(req.patterns, req.one_shot, req.models)

            evt_relay = EvtRelay(self, evt, self.kb)

            prev_nb_cb = len(self.kb.eventsubscriptions.setdefault(evt, set()))

            self.kb.eventsubscriptions[evt].add(evt_relay)

            if len(self.kb.eventsubscriptions[evt]) > prev_nb_cb:
                self.get_logger().warn("Subscribed to event " + evt)
            else:
                self.get_logger().info(
                    "Event %s already subscribed to. Nothing to do." % evt)

            response.id = evt
            response.topic = self.get_namespace() + "/" + EVENTS_TOPIC_NS + evt
            return response

        except KbServerError as kbe:
            self.get_logger().error(
                f"Unable to create event! Original error was: {str(kbe)}")
            response.id = ""
            return response

    def _register_with_gateway(self):
        if not self._register_client.service_is_ready():
            return

        self._register_timer.cancel()

        req = RegisterModule.Request()
        req.type = 'Query_Request'
        req.module_name = 'knowledge_core'
        future = self._register_client.call_async(req)
        future.add_done_callback(self._on_gateway_register_response)

    def _on_gateway_register_response(self, future):
        try:
            response = future.result()
            if response.topic:
                self.get_logger().info(
                    f"Registered with ca_gateway for type 'Query_Request' on topic: {response.topic}")
                self._query_sub = self.create_subscription(
                    LocalGenericMessage, response.topic,
                    self._on_inter_agent_query, 10)
            else:
                self.get_logger().warn("ca_gateway registration returned an empty topic")
        except Exception as e:
            self.get_logger().error(f"ca_gateway registration failed: {e}")

    def _on_inter_agent_query(self, msg):
        sender = msg.agents[0] if msg.agents else ''
        try:
            req = deserialize_message(bytes(msg.data), Query.Request)
        except Exception as e:
            self.get_logger().error(
                f"Failed to deserialize inter-agent query from '{sender}': {e}")
            return

        future = self._local_query_client.call_async(req)
        future.add_done_callback(lambda f: self._on_query_response(f, sender))

    def _on_query_response(self, future, sender):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(
                f"Local query service call failed for sender '{sender}': {e}")
            return

        reply = LocalGenericMessage()
        reply.agents = [sender]
        reply.type = 'Query.Response'
        reply.data = list(serialize_message(response))
        self._gateway_out_pub.publish(reply)

        self.get_logger().info(
            f"Answered inter-agent query from '{sender}': success={response.success}"
        )

    def step(self):
        if not hasattr(self, 'last_diagnostics_ts'):
            return

        now = self.get_clock().now()

        if (now - self.last_diagnostics_ts).nanoseconds > 1e9 / DIAGNOSTICS_FREQUENCY:
            msg = DiagnosticStatus(
                name="/reasoning/kb/knowledge_core",
                values=[
                    KeyValue(key="Module name", value="knowledge_core"),
                    KeyValue(key="Last facts", value="; ".join([f for f in self.last_facts])),
                ]
            )
            if self.kb.reasoner_enabled:
                msg.level = DiagnosticStatus.OK
                msg.message = "Knowledge base running, with OWL/RDF reasoner enabled"
            else:
                msg.level = DiagnosticStatus.WARN
                msg.message = "Knowledge base running, but OWL/RDF reasoner not enabled"
            arr = DiagnosticArray()
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.status = [msg]
            self.diagnostics_pub.publish(arr)

            self.last_diagnostics_ts = now

        active_concepts = set(self.kb.active_concepts)
        if active_concepts != self._current_active_concepts:
            active_concepts_msg = ActiveConcepts()

            for c in active_concepts:
                active_concepts_msg.concepts.append(c)

            self.active_concepts_pub.publish(active_concepts_msg)
            self._current_active_concepts = active_concepts

        rclpy.spin_once(self, timeout_sec=0)

    def shutdown(self):

        self.get_logger().info("KnowledgeCore closing")
        self.destroy_node()
        rclpy.shutdown()
