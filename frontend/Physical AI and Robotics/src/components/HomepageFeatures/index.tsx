import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  icon: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Hands-On Learning',
    icon: '🛠️',
    description: (
      <>
        Build real robotic systems from day one. Every concept is reinforced with
        practical projects and exercises using ROS2 and industry-standard tools.
      </>
    ),
  },
  {
    title: 'Industry-Relevant Skills',
    icon: '🚀',
    description: (
      <>
        Learn the exact technologies used by leading robotics companies: ROS2,
        NVIDIA Isaac, Digital Twins, and modern AI frameworks.
      </>
    ),
  },
  {
    title: 'AI-Native Approach',
    icon: '🤖',
    description: (
      <>
        Master Vision-Language-Action models and foundation models for robotics.
        Build intelligent systems that understand and act in the physical world.
      </>
    ),
  },
];

function Feature({title, icon, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <div className={styles.featureIcon} aria-hidden="true">{icon}</div>
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
