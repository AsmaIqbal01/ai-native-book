import React from 'react';
import Link from '@docusaurus/Link';
import { useTranslation } from '../../contexts/TranslationContext';

/**
 * Custom Footer Component with Translation Support
 *
 * Overrides the default Docusaurus footer to support language switching
 */

export default function Footer(): JSX.Element {
  const { t, language } = useTranslation();
  const currentYear = new Date().getFullYear();

  return (
    <footer
      className="footer footer--dark"
      style={{
        fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit',
      }}
    >
      <div className="container container-fluid">
        <div className="row footer__links">
          {/* Learn Section */}
          <div className="col footer__col">
            <div className="footer__title">{t('footer.learn')}</div>
            <ul className="footer__items clean-list">
              <li className="footer__item">
                <Link className="footer__link-item" to="/docs/introduction">
                  {t('footer.getStarted')}
                </Link>
              </li>
              <li className="footer__item">
                <Link className="footer__link-item" to="/docs/chapter1/physical-ai">
                  {t('footer.physicalAI')}
                </Link>
              </li>
              <li className="footer__item">
                <Link className="footer__link-item" to="/docs/chapter2/ros2-architecture">
                  {t('footer.ros2Basics')}
                </Link>
              </li>
            </ul>
          </div>

          {/* Community Section */}
          <div className="col footer__col">
            <div className="footer__title">{t('footer.community')}</div>
            <ul className="footer__items clean-list">
              <li className="footer__item">
                <a
                  className="footer__link-item"
                  href="https://github.com/AsmaIqbal01/ai-native-book"
                  target="_blank"
                  rel="noopener noreferrer"
                >
                  {t('footer.github')}
                </a>
              </li>
              <li className="footer__item">
                <a
                  className="footer__link-item"
                  href="https://discordapp.com/invite/docusaurus"
                  target="_blank"
                  rel="noopener noreferrer"
                >
                  {t('footer.discord')}
                </a>
              </li>
              <li className="footer__item">
                <a
                  className="footer__link-item"
                  href="https://twitter.com/docusaurus"
                  target="_blank"
                  rel="noopener noreferrer"
                >
                  {t('footer.twitter')}
                </a>
              </li>
            </ul>
          </div>

          {/* More Section */}
          <div className="col footer__col">
            <div className="footer__title">{t('footer.more')}</div>
            <ul className="footer__items clean-list">
              <li className="footer__item">
                <Link className="footer__link-item" to="/docs/resources/references">
                  {t('footer.resources')}
                </Link>
              </li>
              <li className="footer__item">
                <a
                  className="footer__link-item"
                  href="https://github.com/AsmaIqbal01/ai-native-book"
                  target="_blank"
                  rel="noopener noreferrer"
                >
                  {t('footer.github')}
                </a>
              </li>
            </ul>
          </div>
        </div>

        {/* Copyright */}
        <div className="footer__bottom text--center">
          <div className="footer__copyright">
            {t('footer.copyright').replace('{year}', currentYear.toString())}
          </div>
        </div>
      </div>
    </footer>
  );
}
