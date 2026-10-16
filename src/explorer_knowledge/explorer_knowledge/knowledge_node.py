"""ROS 2 Node exposing the Knowledge Base and Inference Engine."""
import rclpy
from rclpy.node import Node
from explorer_interfaces.msg import SymbolicFact
from explorer_interfaces.srv import AddSymbolicFact
from explorer_knowledge.fact import EpistemicStatus, Fact
from explorer_knowledge.inference_engine import InferenceEngine, create_default_robotics_rules
from explorer_knowledge.knowledge_base import KnowledgeBase


class KnowledgeNode(Node):
    """ROS 2 node managing symbolic knowledge and running inference rules."""

    def __init__(self) -> None:
        super().__init__('knowledge_node')
        self.kb = KnowledgeBase()
        self.engine = InferenceEngine(self.kb)

        # Register default rules
        for rule in create_default_robotics_rules():
            self.engine.register_rule(rule)

        # Service to insert facts
        self.srv_add_fact = self.create_service(
            AddSymbolicFact,
            '~/add_fact',
            self._handle_add_fact
        )

        # Periodic inference timer (e.g. 2 Hz)
        self.timer = self.create_timer(0.5, self._on_timer)

        self.get_logger().info('KnowledgeNode initialized with default robotics rules.')

    def _handle_add_fact(
        self,
        request: AddSymbolicFact.Request,
        response: AddSymbolicFact.Response
    ) -> AddSymbolicFact.Response:
        try:
            status_str = request.fact.epistemic_status or "OBSERVED"
            fact = Fact(
                predicate=request.fact.predicate,
                arguments=request.fact.arguments,
                confidence=request.fact.confidence,
                evidence_source=request.fact.evidence_source,
                is_negated=request.fact.is_negated,
                epistemic_status=EpistemicStatus(status_str),
            )
            updated = self.kb.assert_fact(fact)
            self.engine.run_to_fixpoint()
            response.success = True
            response.message = f"Fact {fact} asserted (updated={updated})"
        except Exception as e:
            response.success = False
            response.message = f"Error asserting fact: {e}"
        return response

    def _on_timer(self) -> None:
        inferred = self.engine.step()
        if inferred > 0:
            self.get_logger().debug(f'Inferred {inferred} new symbolic facts.')


def main(args=None):
    rclpy.init(args=args)
    node = KnowledgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()