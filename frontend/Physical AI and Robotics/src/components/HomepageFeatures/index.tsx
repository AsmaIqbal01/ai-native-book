import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  image: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI Fundamentals',
    image: require('@site/static/img/embeddedIntelligence.png').default,
    description: (
      <>
        Master the foundations of embodied intelligence - from digital AI to physical robots
        that understand and interact with the real world.
      </>
    ),
  },
  {
    title: 'Hands-On with ROS2 & Gazebo',
    image: require('@site/static/img/ros2.png').default,
    description: (
      <>
        Build real robotic systems using ROS2, simulate with Gazebo & Unity,
        and deploy to actual hardware. Includes complete code examples.
      </>
    ),
  },
  {
    title: 'AI-Powered Robotics',
    image: require('@site/static/img/Isaac.png').default,
    description: (
      <>
        Integrate NVIDIA Isaac, Vision-Language-Action models, and cutting-edge AI
        to create intelligent humanoid robots.
      </>
    ),
  },
];

function Feature({title, image, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')} role="article" aria-labelledby={`feature-title-${title.toLowerCase().replace(/\s+/g, '-')}`}>
      <div className="text--center">
        <img
          src={image}
          className={styles.featureSvg}
          alt={`${title} illustration`}
        />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3" id={`feature-title-${title.toLowerCase().replace(/\s+/g, '-')}`}>
          {title}
        </Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features} aria-labelledby="features-title">
      <div className="container">
        <Heading as="h2" id="features-title" className="visually-hidden">
          Key Features of the Course
        </Heading>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
