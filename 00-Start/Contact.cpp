#include "Contact.h"

void Contact::ResolveContact(Contact& contact) {
	Body* a = contact.a;
	Body* b = contact.b;

	const float invMassA = a->inverseMass;
	const float invMassB = b->inverseMass;

	const float elasticityA = a->elasticity;
	const float elasticityB = b->elasticity;
	const float elasticity = elasticityA * elasticityB;


	// collision impulse
	const Vec3& n = contact.normal;
	const Vec3& velAb = a->linearVelocity - b->linearVelocity;
	const float impulseValueJ = -(1.0f + elasticity) * velAb.Dot(n) / (invMassA + invMassB);
	const Vec3 impulse = n * impulseValueJ;

	a->ApplyImpulseLinear(impulse);
	b->ApplyImpulseLinear(impulse * -1.0f);

	const float tA = a->inverseMass / (a->inverseMass + b->inverseMass);
	const float tB = b->inverseMass / (a->inverseMass + b->inverseMass);
	const Vec3 d = contact.ptOnBWorldSpace - contact.ptOnAWorldSpace;

	a->position += d * tA;
	b->position -= d * tB;
}

